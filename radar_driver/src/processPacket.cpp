/* General Use Includes */
#include "processPacket.h"
#include <cstring>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>

//Processing Rules: Written out in better detail in the spec doc
//Spec Doc: https://docs.google.com/document/d/13W7H_Z5pgCx8rVgnv7u1MaKYaGQvnPspWVuuTmzJgUA/edit

//#define PROCESS_SS_PACKET //Perception doesn't need it

static PacketGroup_t EmptyPackets; //Local static only, Cleanup struct

//Some local functions
bool loadRDIMessageFromPacket(radar_driver::msg::RadarPacket* newMsg, const radar_driver::msg::RadarPacket::SharedPtr oldMsg);
void loadSSMessage(radar_driver::msg::SensorStatus* msg, SSPacket_t* packet);

PacketProcessor::PacketProcessor() {
    Mutex = PTHREAD_MUTEX_INITIALIZER; //Non recursive
    Publisher = NULL; //Publisher object for callback
    publish = false; //Publishing flag
    curNearTimeStamp = 0, curFarTimeStamp = 0;
    curNearIdx = 0, curFarIdx = 0;
    scanMode = NEAR_FAR_SCAN; // 0=Near&Far, 1=NearOnly, 2=FarOnly
}

/* Initialization Processor
*
*/
uint8_t PacketProcessor::initializePacketProcessor(RadarPublisher* newPublisher, uint8_t newscanMode) {
    pthread_mutex_lock(&Mutex);

    if (clearAllPackets()) { //Wipe internal struct
        pthread_mutex_unlock(&Mutex);
        return INIT_FAIL;
    }

    scanMode = newscanMode;
    Publisher = newPublisher;
    //All packet structs should be 0'd by default
    pthread_mutex_unlock(&Mutex);

    return SUCCESS;
}

uint8_t PacketProcessor::processRDIMsg(const radar_driver::msg::RadarPacket::SharedPtr packet) {
    if ( packet->detections.size() < 1 ){
        return NO_DETECTIONS;
    }

    //Only mutex here b/c not using the Packets struct yet
    pthread_mutex_lock(&Mutex);

    if(packet->event_id == FAR1 || packet->event_id == FAR0) {
        if (scanMode != NEAR_SCAN) {
            if (curFarTimeStamp == 0){ //Init Case
                curFarTimeStamp = packet->time_stamp;
            } else if (curFarTimeStamp != packet->time_stamp) { //New timestamp
                curFarIdx = (curFarIdx + 1) % 2;
                curFarTimeStamp = packet->time_stamp;

                if (scanMode == FAR_SCAN || curFarIdx == curNearIdx) { //Indexes are the same
                    // We just got a new TS for both near & far, so we should publish the old buffer
                    publish = true;
                }
            }

            PacketGroup_t * curGroup = &PacketsBuffer[curFarIdx]; //Tmp to make code easier to read
            if (curGroup->numFarPackets >= NUM_FAR) {
                pthread_mutex_unlock(&Mutex);
                return PACKET_GROUP_FULL;
            }

            if (loadRDIMessageFromPacket(&curGroup->farPackets[curGroup->numFarPackets], packet)) {
                curGroup->numFarPackets++;
            }

            if (publish) {
                uint8_t err = publishPackets((curFarIdx+1)%2); //Publish the previous buffer idx
                publish = false;
                pthread_mutex_unlock(&Mutex);
                return err;
            }
        }
    } else if (packet->event_id == NEAR0 || packet->event_id == NEAR1 || packet->event_id == NEAR2) {
        if (scanMode != FAR_SCAN) {
            if (curNearTimeStamp == 0){ //Init Case
                curNearTimeStamp = packet->time_stamp;
            } else if (curNearTimeStamp != packet->time_stamp) { //New timestamp
                curNearIdx = (curNearIdx + 1) % 2;
                curNearTimeStamp = packet->time_stamp;

                if (scanMode == NEAR_SCAN || curNearIdx == curFarIdx) { //Indexes are the same
                    // We just got a new TS for both near & far, so we should publish the old buffer
                    publish = true;
                }
            }

            PacketGroup_t* curGroup = &PacketsBuffer[curNearIdx]; //Tmp to make code easier to read
            if (curGroup->numNearPackets >= NUM_NEAR) {
                pthread_mutex_unlock(&Mutex);
                return PACKET_GROUP_FULL;
            }

            if (loadRDIMessageFromPacket(&curGroup->nearPackets[curGroup->numNearPackets], packet)) {
                curGroup->numNearPackets++;
            }

            if (publish) {
                uint8_t err = publishPackets((curNearIdx+1)%2); //Publish the previous buffer idx
                publish = false;
                pthread_mutex_unlock(&Mutex);
                return err;
            }
        }
    } else {
        pthread_mutex_unlock(&Mutex);
        return BAD_EVENT_ID;
    }

    pthread_mutex_unlock(&Mutex);
    return SUCCESS;
}

/* Process Sensor Status Packet
* -- Add packet to our internal struct
* -- Call our publisher to publish all packets since we have a group
* -- Publish as soon as we have the sensor status packet
*/
uint8_t PacketProcessor::processSSPacket(SSPacket_t* packet) {
    // TODO: Raise an alarm on the dashboard with error info.
    if (packet->Defective & DEFECTIVE_HW) {
        return SS_DEFECTIVE_HW;
    } else if (packet->BadSupplyVolt & BAD_VOLTAGE) {
        return SS_BAD_VOLT;
    } else if (packet->BadTemp & BAD_TEMP) {
        return SS_BAD_TEMP;
    } else if (packet->GmMissing & GM_MISSING) {
        return SS_GM_MISSING;
    } else if (packet->TxPowerStatus & POWER_REDUCED) {
        return SS_PWR_REDUCED;
    }
    pthread_mutex_lock(&Mutex);

    //TODO: Implement Error Checking Here...

#if PROCESS_SS_PACKET
    loadSSMessage(&Packets.sensorStatusMsg, packet);
#endif

    pthread_mutex_unlock(&Mutex);
    return SUCCESS;
}

/* Publish Packets
* -- Send our packet struct to the ROS publisher
* -- Clear our intial struct & reset
* -- Only called ffrom synchronized methods, dont need to mutex
*/
uint8_t PacketProcessor::publishPackets(uint8_t idx) {

    if (Publisher == NULL) { //Publisher not set up
        //Check if we can still clear the packets
        if (clearPackets(idx)) { //Everything failed
            return NO_PUB_CLR_FAIL;
        }
        return NO_PUBLISHER; //Only null publisher, cleared ok
    } else if (Publisher->pubCallback(&PacketsBuffer[idx])) {
        return PUBLISH_FAIL;
    }

    if (clearPackets(idx)) {
        return CLEAR_FAIL;
    }
    return SUCCESS;
}

/* Clear Packet Struct
* -- Empty one internal packet struct
* -- Only called form synchronized methods, dont need to mutex
*/
uint8_t PacketProcessor::clearPackets(uint8_t idx) {
    if (idx < 2) {
        PacketsBuffer[idx] = EmptyPackets; //Replace w/ empty struct
        PacketsBuffer[idx].numFarPackets = 0;
        PacketsBuffer[idx].numNearPackets = 0;
        return SUCCESS;
    }
    return CLEAR_FAIL;
}

/* Clear All Packet Buffers
* -- Empty both packet structs
* -- Only called form synchronized methods, dont need to mutex
*/
uint8_t PacketProcessor::clearAllPackets() {
    clearPackets(0);
    clearPackets(1);

    return SUCCESS;
}

bool loadRDIMessageFromPacket(radar_driver::msg::RadarPacket* newMsg, const radar_driver::msg::RadarPacket::SharedPtr oldMsg) {
    newMsg->event_id                    = oldMsg->event_id;
    newMsg->time_stamp                  = oldMsg->time_stamp;
    newMsg->measurement_counter         = oldMsg->measurement_counter;
    newMsg->vambig                      = oldMsg->vambig;
    newMsg->center_frequency            = oldMsg->center_frequency;
    newMsg->detections.clear();

    for(uint8_t i = 0; i < oldMsg->detections.size(); i++) {

        // TODO: Figure out an SNR threshold value that actually works here.
        if (oldMsg->detections[i].snr < SNR_THRESHOLD) { // Too much noise; drop detection.
            continue;
        } else if (abs(oldMsg->detections[i].vrel_rad) < VELOCITY_LOWER_THRESHOLD) {
            continue;
        } else if (oldMsg->detections[i].pos_x > DISTANCE_MAX_THRESHOLD) { // need to do trig
            continue;
        } else if (oldMsg->detections[i].pos_x < DISTANCE_MIN_THRESHOLD) {
            continue;
        } else if (oldMsg->detections[i].rcs0 > RCS_THRESHOLD) {
            continue;
        } else if (oldMsg->event_id == FAR1 || oldMsg->event_id == FAR0) {
            //limit far scan to 9 degrees, according to conti
            if (oldMsg->detections[i].az_ang0 < -0.15708 || oldMsg->detections[i].az_ang1 > 0.15708) {
                continue;
            }
        } else if(oldMsg->detections[i].az_ang0 < AZI_ANGLE_0_THRESHOLD || oldMsg->detections[i].az_ang1 > AZI_ANGLE_1_THRESHOLD){
            continue;
        }

        radar_driver::msg::RadarDetection data;

        // Copy all data from old message detection to new message

        data.pdh0          = oldMsg->detections[i].pdh0;

        data.az_ang0       = oldMsg->detections[i].az_ang0;
        data.rcs0          = oldMsg->detections[i].rcs0;
        data.az_ang_var0   = oldMsg->detections[i].az_ang_var0;
        data.prob0         = oldMsg->detections[i].prob0;

        data.az_ang1       = oldMsg->detections[i].az_ang1;
        data.rcs1          = oldMsg->detections[i].rcs1;
        data.az_ang_var1   = oldMsg->detections[i].az_ang_var1;
        data.prob1         = oldMsg->detections[i].prob1;

        data.vrel_rad     = oldMsg->detections[i].vrel_rad;
        data.el_ang       = oldMsg->detections[i].el_ang;
        data.range_var    = oldMsg->detections[i].range_var;
        data.vrel_rad_var = oldMsg->detections[i].vrel_rad_var;
        data.el_ang_var   = oldMsg->detections[i].el_ang_var;
        data.snr          = oldMsg->detections[i].snr;
        data.range        = oldMsg->detections[i].range;

        data.pos_x = oldMsg->detections[i].pos_x;
        data.pos_y = oldMsg->detections[i].pos_y;
        data.pos_z = oldMsg->detections[i].pos_z;

        newMsg->detections.push_back(data);
    }

    // Return true if there is at least one detection in newMsg after filtering
    return (newMsg->detections.size() > 0);
}

/* Load SS ROS Message
* -- Local Function, not in class definition
*/
void loadSSMessage(radar_driver::msg::SensorStatus* msg, SSPacket_t* packet) {
    msg->part_number            = packet->PartNumber;
    msg->assembly_part_number   = packet->AssemblyPartNumber;
    msg->sw_part_number         = packet->SWPartNumber;
    for (uint8_t i = 0; i < SENSOR_SERIAL_NUM_LEN; i++) {
        msg->serial_number[i]   = packet->SerialNumber[i];
    }
    msg->bl_version             = packet->BLVersion;
    msg->sw_version             = packet->SWVersion;
    msg->utc_time_stamp         = packet->UTCTimeStamp;
    msg->time_stamp             = packet->TimeStamp;
    msg->surface_damping        = packet->SurfaceDamping;
    msg->op_state               = packet->OpState;
    msg->current_far_cf         = packet->CurrentFarCF;
    msg->current_near_cf        = packet->CurrentNearCF;
    msg->defective              = packet->Defective;
    msg->bad_supply_volt        = packet->BadSupplyVolt;
    msg->bad_temp               = packet->BadTemp;
    msg->gm_missing             = packet->GmMissing;
    msg->tx_power_status        = packet->TxPowerStatus;
    msg->maximum_range_far      = packet->MaxRangeFar;
    msg->maximum_range_near     = packet->MaxRangeNear;
}
/* Set ROS publisher callback
*/
void PacketProcessor::setPublisherCallback(RadarPublisher* newPublisher) {
    pthread_mutex_lock(&Mutex);
    Publisher = newPublisher;
    pthread_mutex_unlock(&Mutex);
}

/* Print the currently selected buffer
*/
void PacketProcessor::printPacketGroup(uint8_t idx) {
    pthread_mutex_lock(&Mutex);
    if (idx >= 2) {
        printf("Improper Idx Given: %u\r\n", idx);
        pthread_mutex_unlock(&Mutex);
        return;
    }
    PacketGroup_t * Packets = &PacketsBuffer[idx];

    for (uint8_t j = 0; j < Packets->numFarPackets; j++) {
        printf("\nPROC:Far Packet: %d, len: %u\n", j, static_cast<unsigned>(Packets->farPackets[j].detections.size()));
        for(uint8_t i = 0; i < Packets->farPackets[j].detections.size(); i++) {
            printf("PROC:RDI Idx: %d \n"    , i);
            printf("PROC:posX %f \n"        , Packets->farPackets[j].detections[i].pos_x);
            printf("PROC:posY %f \n"        , Packets->farPackets[j].detections[i].pos_y);
            printf("PROC:posZ %f \n"        , Packets->farPackets[j].detections[i].pos_z);
            printf("PROC:VrelRad %f \n"     , Packets->farPackets[j].detections[i].vrel_rad);
            printf("PROC:AzAng0 %f \n"      , Packets->farPackets[j].detections[i].az_ang0);
            printf("PROC:AzAng1 %f \n"      , Packets->farPackets[j].detections[i].az_ang1);
            printf("PROC:ElAng %f \n"       , Packets->farPackets[j].detections[i].el_ang);
            printf("PROC:RCS0 %f \n"        , Packets->farPackets[j].detections[i].rcs0);
            printf("PROC:RCS1 %f \n"        , Packets->farPackets[j].detections[i].rcs1);
            printf("PROC:RangeVar %f \n"    , Packets->farPackets[j].detections[i].range_var);
            printf("PROC:VrelRadVar %f \n"  , Packets->farPackets[j].detections[i].vrel_rad_var);
            printf("PROC:AzAngVar0 %f \n"   , Packets->farPackets[j].detections[i].az_ang_var0);
            printf("PROC:AzAngVar1 %f \n"   , Packets->farPackets[j].detections[i].az_ang_var1);
            printf("PROC:ElAngVar %f \n"    , Packets->farPackets[j].detections[i].el_ang_var);
            printf("PROC:Prob0 %f \n"       , Packets->farPackets[j].detections[i].prob0);
            printf("PROC:Prob1 %f \n"       , Packets->farPackets[j].detections[i].prob1);
            printf("PROC:SNR %f \n"         , Packets->farPackets[j].detections[i].snr);
            printf("\n");
        }
    }

    for (uint8_t j = 0; j < Packets->numNearPackets; j++) {
        printf("PROC:Near Packet: %d, len: %u\n", j, static_cast<unsigned>(Packets->nearPackets[j].detections.size()));
        for(uint8_t i = 0; i < Packets->nearPackets[j].detections.size(); i++) {
            printf("PROC:RDI Idx: %d \n"    , i);
            printf("PROC:posX %f \n"        , Packets->nearPackets[j].detections[i].pos_x);
            printf("PROC:posY %f \n"        , Packets->nearPackets[j].detections[i].pos_y);
            printf("PROC:posZ %f \n"        , Packets->nearPackets[j].detections[i].pos_z);
            printf("PROC:VrelRad %f \n"     , Packets->nearPackets[j].detections[i].vrel_rad);
            printf("PROC:AzAng0 %f \n"      , Packets->nearPackets[j].detections[i].az_ang0);
            printf("PROC:AzAng1 %f \n"      , Packets->nearPackets[j].detections[i].az_ang1);
            printf("PROC:ElAng %f \n"       , Packets->nearPackets[j].detections[i].el_ang);
            printf("PROC:RCS0 %f \n"        , Packets->nearPackets[j].detections[i].rcs0);
            printf("PROC:RCS1 %f \n"        , Packets->nearPackets[j].detections[i].rcs1);
            printf("PROC:RangeVar %f \n"    , Packets->nearPackets[j].detections[i].range_var);
            printf("PROC:VrelRadVar %f \n"  , Packets->nearPackets[j].detections[i].vrel_rad_var);
            printf("PROC:AzAngVar0 %f \n"   , Packets->nearPackets[j].detections[i].az_ang_var0);
            printf("PROC:AzAngVar1 %f \n"   , Packets->nearPackets[j].detections[i].az_ang_var1);
            printf("PROC:ElAngVar %f \n"    , Packets->nearPackets[j].detections[i].el_ang_var);
            printf("PROC:Prob0 %f \n"       , Packets->nearPackets[j].detections[i].prob0);
            printf("PROC:Prob1 %f \n"       , Packets->nearPackets[j].detections[i].prob1);
            printf("PROC:SNR %f \n"         , Packets->nearPackets[j].detections[i].snr);
            printf("\n");
        }
    }

    pthread_mutex_unlock(&Mutex);
}
