/* General Use Includes */
#include "resMultipliers.h"
#include "processPacket.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <inttypes.h>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "radar_driver/msg/radar_packet.hpp"
#include "radar_driver/msg/sensor_status.hpp"

#define DETECTIONS_IN_ELEMENT_BYTE_POS 55
#define PARSE_SS

//Resolution Multipliers, make static const global since never changing
static const resMultipliersRDI_t resRDIMults;
static const resMultipliersSS_t  resSSMults;

//Unfiltered ROS publisher
static rclcpp::Publisher<radar_driver::msg::RadarPacket>::SharedPtr unfilteredPublisher;

static const size_t UDP_HEADER_LEN = 8;
static const size_t RDI_PAYLOAD_HEADER_END = 56;
static const size_t RDI_DETECTION_LEN = 28;
static const size_t SS_PACKET_MIN_LEN = 121;

void initUnfilteredPublisher(rclcpp::Node::SharedPtr nh) {
    unfilteredPublisher = nh->create_publisher<radar_driver::msg::RadarPacket>("unfiltered_radar_packet", 50);
}

void resetUnfilteredPublisher() {
    unfilteredPublisher.reset();
}

uint8_t parse_packet(udphdr_t* udphdr, unsigned char* packetptr, size_t packetLen) {

    if (packetLen < 24) {
        return BAD_PACKET_SIZE;
    }

    size_t udpLen = ntohs(udphdr->len);
    if (udpLen < UDP_HEADER_LEN || udpLen > packetLen) {
        return BAD_PACKET_SIZE;
    }

    const size_t availableLen = udpLen;

    packetHeader_t Header; //Create the header locally

    //128 bit (16 Byte) Packet Header
    //Were not really using this, decoding for completion's sake
    Header.service_ID      = (packetptr[8] << 8) | (packetptr[9]); //Big Endian
    Header.event_ID        = (packetptr[10] << 8) | packetptr[11]; // Indicates far or near scan
    Header.length          = (packetptr[12] << 24) | (packetptr[13] << 16) | (packetptr[14] << 8) | packetptr[15]; //8 bytes past this
    Header.request_ID      = (packetptr[16] << 24) | (packetptr[17] << 16) | (packetptr[18] << 8) | packetptr[19];
    Header.protocol_ver    =  packetptr[20];
    Header.interface_ver   =  packetptr[21];
    Header.message_Type    =  packetptr[22];
    Header.return_Code     =  packetptr[23];

    if (Header.service_ID == RDI_PACKET_ID) {
        if (availableLen < RDI_PAYLOAD_HEADER_END) {
            return BAD_PACKET_SIZE;
        }

        if (packetptr[DETECTIONS_IN_ELEMENT_BYTE_POS] == 0) {
            return NO_DETECTIONS;
        }

        if (packetptr[DETECTIONS_IN_ELEMENT_BYTE_POS] > RDI_ARRAY_LEN) {
            return BAD_PACKET_SIZE;
        }

        if (availableLen < RDI_PAYLOAD_HEADER_END + (packetptr[DETECTIONS_IN_ELEMENT_BYTE_POS] * RDI_DETECTION_LEN)) {
            return BAD_PACKET_SIZE;
        }

        RDIPacket_t RDI_Packet; //Create RDI Packet struct

        //256 Bit (32 Byte) Payload Header
        RDI_Packet.payloadHeaderData.EventID            = Header.event_ID; //Duplicated b/c useful later
        RDI_Packet.payloadHeaderData.CRC                = (packetptr[24] << 8) | (packetptr[25]);     
        RDI_Packet.payloadHeaderData.Len                = (packetptr[26] << 8) | (packetptr[27]);           
        RDI_Packet.payloadHeaderData.SQC                =  packetptr[28];
        RDI_Packet.payloadHeaderData.MessageCounter     =  packetptr[29];
        RDI_Packet.payloadHeaderData.UTCTimeStamp       = (static_cast<uint64_t>(packetptr[30]) << 56) | (static_cast<uint64_t>(packetptr[31]) << 48) | (static_cast<uint64_t>(packetptr[32]) << 40) | (static_cast<uint64_t>(packetptr[33]) << 32) | (static_cast<uint64_t>(packetptr[34]) << 24) | (static_cast<uint64_t>(packetptr[35]) << 16) | (static_cast<uint64_t>(packetptr[36]) << 8) | static_cast<uint64_t>(packetptr[37]);
        RDI_Packet.payloadHeaderData.TimeStamp          = (packetptr[38] << 24) | (packetptr[39] << 16) | (packetptr[40] << 8) | packetptr[41];
        RDI_Packet.payloadHeaderData.MeasurementCounter = (packetptr[42] << 24) | (packetptr[43] << 16) | (packetptr[44] << 8) | packetptr[45];
        RDI_Packet.payloadHeaderData.CycleCounter       = (packetptr[46] << 24) | (packetptr[47] << 16) | (packetptr[48] << 8) | packetptr[49];
        RDI_Packet.payloadHeaderData.NofDetections      = (packetptr[50] << 8) | packetptr[51];
        RDI_Packet.payloadHeaderData.VambigRaw          = (packetptr[52] << 8) | packetptr[53];
        RDI_Packet.payloadHeaderData.CenterFrequencyRaw =  packetptr[54];
        RDI_Packet.payloadHeaderData.DetectionsInPacket =  packetptr[55];
        RDI_Packet.payloadHeaderData.Vambig             = RDI_Packet.payloadHeaderData.VambigRaw            * resRDIMults.VambigRes;
        RDI_Packet.payloadHeaderData.CenterFrequency    = RDI_Packet.payloadHeaderData.CenterFrequencyRaw   * resRDIMults.CenterFreqRes;

        //7168 - 8512 Bit (896 - 1064 Byte) Radar Dection Payload. Size depends on eventID. 
        int listStartIndex = 56;
        auto readNextU16 = [&]() {
            uint16_t value = (packetptr[listStartIndex] << 8) | packetptr[listStartIndex + 1];
            listStartIndex += 2;
            return value;
        };
        for (int i = 0; i < RDI_Packet.payloadHeaderData.DetectionsInPacket; i++) { //Only fill as many as we have valid packets, dont bother filling 0's

            //Radar Detection Images are 224 Bits (28 Bytes) each
            RDI_Packet.listDataArray[i].f_Range      = readNextU16();
            RDI_Packet.listDataArray[i].f_VrelRad    = readNextU16();
            RDI_Packet.listDataArray[i].f_AzAng0     = readNextU16();
            RDI_Packet.listDataArray[i].f_AzAng1     = readNextU16();
            RDI_Packet.listDataArray[i].f_ElAng      = readNextU16();
            RDI_Packet.listDataArray[i].f_RCS0       = readNextU16();
            RDI_Packet.listDataArray[i].f_RCS1       = readNextU16();
            RDI_Packet.listDataArray[i].f_Prob0      =  packetptr[listStartIndex++];
            RDI_Packet.listDataArray[i].f_Prob1      =  packetptr[listStartIndex++];
            RDI_Packet.listDataArray[i].f_RangeVar   = readNextU16();
            RDI_Packet.listDataArray[i].f_VrelRadVar = readNextU16();
            RDI_Packet.listDataArray[i].f_AzAngVar0  = readNextU16();
            RDI_Packet.listDataArray[i].f_AzAngVar1  = readNextU16();
            RDI_Packet.listDataArray[i].f_ElAngVar   = readNextU16();
            RDI_Packet.listDataArray[i].ui_Pdh0      =  packetptr[listStartIndex++];
            RDI_Packet.listDataArray[i].f_SNR        =  packetptr[listStartIndex++];


            //Add in the proper resolution
            RDI_Packet.resListDataArray[i].f_Range      = RDI_Packet.listDataArray[i].f_Range      * resRDIMults.f_RangeRes;
            RDI_Packet.resListDataArray[i].f_VrelRad    = RDI_Packet.listDataArray[i].f_VrelRad    * resRDIMults.f_VrelRadRes;
            RDI_Packet.resListDataArray[i].f_AzAng0     = RDI_Packet.listDataArray[i].f_AzAng0     * resRDIMults.f_AzAng0Res;
            RDI_Packet.resListDataArray[i].f_AzAng1     = RDI_Packet.listDataArray[i].f_AzAng1     * resRDIMults.f_AzAng1Res;
            RDI_Packet.resListDataArray[i].f_ElAng      = RDI_Packet.listDataArray[i].f_ElAng      * resRDIMults.f_ElAngRes;
            RDI_Packet.resListDataArray[i].f_RCS0       = RDI_Packet.listDataArray[i].f_RCS0       * resRDIMults.f_RCS0Res;
            RDI_Packet.resListDataArray[i].f_RCS1       = RDI_Packet.listDataArray[i].f_RCS1       * resRDIMults.f_RCS1Res;
            RDI_Packet.resListDataArray[i].f_Prob0      = RDI_Packet.listDataArray[i].f_Prob0      * resRDIMults.f_Prob0Res;
            RDI_Packet.resListDataArray[i].f_Prob1      = RDI_Packet.listDataArray[i].f_Prob1      * resRDIMults.f_Prob1Res;
            RDI_Packet.resListDataArray[i].f_RangeVar   = RDI_Packet.listDataArray[i].f_RangeVar   * resRDIMults.f_RangeVarRes;
            RDI_Packet.resListDataArray[i].f_VrelRadVar = RDI_Packet.listDataArray[i].f_VrelRadVar * resRDIMults.f_VrelRadVarRes;
            RDI_Packet.resListDataArray[i].f_AzAngVar0  = RDI_Packet.listDataArray[i].f_AzAngVar0  * resRDIMults.f_AzAngVar0Res;
            RDI_Packet.resListDataArray[i].f_AzAngVar1  = RDI_Packet.listDataArray[i].f_AzAngVar1  * resRDIMults.f_AzAngVar1Res;
            RDI_Packet.resListDataArray[i].f_ElAngVar   = RDI_Packet.listDataArray[i].f_ElAngVar   * resRDIMults.f_ElAngVarRes;
            RDI_Packet.resListDataArray[i].ui_Pdh0      = RDI_Packet.listDataArray[i].ui_Pdh0;
            RDI_Packet.resListDataArray[i].f_SNR        = RDI_Packet.listDataArray[i].f_SNR        * resRDIMults.f_SNRRes;
        }
    
        //Send packet to get published
        publishRDIPacket(&RDI_Packet);

    } else if (Header.service_ID == SS_PACKET_ID) {
#ifdef PARSE_SS
        if (availableLen < SS_PACKET_MIN_LEN) {
            return BAD_PACKET_SIZE;
        }

        SSPacket_t SS_Packet;

        SS_Packet.CRC                 = (packetptr[24] << 8) | (packetptr[25]);
        SS_Packet.len                 = (packetptr[26] << 8) | (packetptr[27]);
        SS_Packet.SQC                 = packetptr[28];
        SS_Packet.PartNumber          = (static_cast<uint64_t>(packetptr[29]) << 56) | (static_cast<uint64_t>(packetptr[30]) << 48) | (static_cast<uint64_t>(packetptr[31]) << 40) | (static_cast<uint64_t>(packetptr[32]) << 32) | (static_cast<uint64_t>(packetptr[33]) << 24) | (static_cast<uint64_t>(packetptr[34]) << 16) | (static_cast<uint64_t>(packetptr[35]) << 8) | static_cast<uint64_t>(packetptr[36]);
        SS_Packet.AssemblyPartNumber  = (static_cast<uint64_t>(packetptr[37]) << 56) | (static_cast<uint64_t>(packetptr[38]) << 48) | (static_cast<uint64_t>(packetptr[39]) << 40) | (static_cast<uint64_t>(packetptr[40]) << 32) | (static_cast<uint64_t>(packetptr[41]) << 24) | (static_cast<uint64_t>(packetptr[42]) << 16) | (static_cast<uint64_t>(packetptr[43]) << 8) | static_cast<uint64_t>(packetptr[44]);
        SS_Packet.SWPartNumber        = (static_cast<uint64_t>(packetptr[45]) << 56) | (static_cast<uint64_t>(packetptr[46]) << 48) | (static_cast<uint64_t>(packetptr[47]) << 40) | (static_cast<uint64_t>(packetptr[48]) << 32) | (static_cast<uint64_t>(packetptr[49]) << 24) | (static_cast<uint64_t>(packetptr[50]) << 16) | (static_cast<uint64_t>(packetptr[51]) << 8) | static_cast<uint64_t>(packetptr[52]);
        
        for (uint8_t i = 0; i < SENSOR_SERIAL_NUM_LEN; i++) {//26 Byte Array of uint8_t's
            SS_Packet.SerialNumber[i]   = packetptr[53+i];
        }
        
        SS_Packet.BLVersion           = (packetptr[79] << 16) | (packetptr[80] << 8) | packetptr[81];
        SS_Packet.BL_CRC              = (packetptr[82] << 24) | (packetptr[83] << 16) | (packetptr[84] << 8) | packetptr[85];
        SS_Packet.SWVersion           = (packetptr[86] << 16) | (packetptr[87] << 8) | packetptr[88];
        SS_Packet.SW_CRC              = (packetptr[89] << 24) | (packetptr[90] << 16) | (packetptr[91] << 8) | packetptr[92];
        SS_Packet.UTCTimeStamp        = (static_cast<uint64_t>(packetptr[93]) << 56) | (static_cast<uint64_t>(packetptr[94]) << 48) | (static_cast<uint64_t>(packetptr[95]) << 40) | (static_cast<uint64_t>(packetptr[96]) << 32) | (static_cast<uint64_t>(packetptr[97]) << 24) | (static_cast<uint64_t>(packetptr[98]) << 16) | (static_cast<uint64_t>(packetptr[99]) << 8) | static_cast<uint64_t>(packetptr[100]);
        SS_Packet.TimeStamp           = (packetptr[101] << 24) | (packetptr[102] << 16) | (packetptr[103] << 8) | packetptr[104];
        SS_Packet.SurfaceDamping_Raw  = (packetptr[105] << 24) | (packetptr[106] << 16) | (packetptr[107] << 8) | packetptr[108];
        SS_Packet.OpState             = packetptr[109];
        SS_Packet.CurrentFarCF        = packetptr[110];
        SS_Packet.CurrentNearCF       = packetptr[111];
        SS_Packet.Defective           = packetptr[112];
        SS_Packet.BadSupplyVolt       = packetptr[113];
        SS_Packet.BadTemp             = packetptr[114];
        SS_Packet.GmMissing           = packetptr[115];
        SS_Packet.TxPowerStatus       = packetptr[116];
        SS_Packet.MaxRangeFar_Raw     = (packetptr[117] << 8) | (packetptr[118]);
        SS_Packet.MaxRangeNear_Raw    = (packetptr[119] << 8) | (packetptr[120]);

        //Apply resolution to relevant members
        SS_Packet.SurfaceDamping      = SS_Packet.SurfaceDamping_Raw * resSSMults.surfaceDampingRes;
        SS_Packet.MaxRangeNear        = SS_Packet.MaxRangeNear_Raw   * resSSMults.MaxRangeNearRes;
        SS_Packet.MaxRangeFar         = SS_Packet.MaxRangeFar_Raw    * resSSMults.MaxRangeFarRes;

        //Don't bother publishing yet since Perception doesn't need it
        //TODO: Figure out who needs these & build the publishing code
#endif
    } else {
        return BAD_SERVICE_ID;
    }

    return SUCCESS;
}

void publishRDIPacket(RDIPacket_t * packet) {
    radar_driver::msg::RadarPacket msg;
    loadPacketMsg(packet, &msg);
    printf("Publishing new message: %d\n", packet->payloadHeaderData.DetectionsInPacket); 
        // Print the payload header data
    printf("Publishing new message: %d\n", packet->payloadHeaderData.DetectionsInPacket);
    printf("EventID: %d\n", packet->payloadHeaderData.EventID);
    printf("CRC: %d\n", packet->payloadHeaderData.CRC);
    printf("Len: %d\n", packet->payloadHeaderData.Len);
    printf("SQC: %d\n", packet->payloadHeaderData.SQC);
    printf("MessageCounter: %d\n", packet->payloadHeaderData.MessageCounter);
    printf("UTCTimeStamp: %" PRIu64 "\n", packet->payloadHeaderData.UTCTimeStamp);
    printf("TimeStamp: %d\n", packet->payloadHeaderData.TimeStamp);
    printf("MeasurementCounter: %d\n", packet->payloadHeaderData.MeasurementCounter);
    printf("CycleCounter: %d\n", packet->payloadHeaderData.CycleCounter);
    printf("NofDetections: %d\n", packet->payloadHeaderData.NofDetections);
    printf("VambigRaw: %d\n", packet->payloadHeaderData.VambigRaw);
    printf("CenterFrequencyRaw: %d\n", packet->payloadHeaderData.CenterFrequencyRaw);
    printf("RadarPortNum: %d\n", packet->payloadHeaderData.RadarPortNum);
    printf("DetectionsInPacket: %d\n", packet->payloadHeaderData.DetectionsInPacket);
    printf("Vambig: %.3f\n", packet->payloadHeaderData.Vambig);
    printf("CenterFrequency: %.3f\n", packet->payloadHeaderData.CenterFrequency);

    // Print the data from the listDataArray (assuming it has a length of RDI_ARRAY_LEN)
    for (int i = 0; i < RDI_ARRAY_LEN; i++) {
        printf("Detection %d: Range = %.2f, VrelRad = %.2f, AzAng0 = %.2f, AzAng1 = %.2f, ElAng = %.2f\n",
               i,
               packet->resListDataArray[i].f_Range,
               packet->resListDataArray[i].f_VrelRad,
               packet->resListDataArray[i].f_AzAng0,
               packet->resListDataArray[i].f_AzAng1,
               packet->resListDataArray[i].f_ElAng);
    }
    unfilteredPublisher->publish(msg);
}

void loadPacketMsg(RDIPacket_t * packet, radar_driver::msg::RadarPacket * msg) {
    msg->event_id                    = packet->payloadHeaderData.EventID;
    msg->time_stamp                  = packet->payloadHeaderData.TimeStamp;
    msg->measurement_counter         = packet->payloadHeaderData.MeasurementCounter;
    msg->vambig                      = packet->payloadHeaderData.Vambig;
    msg->center_frequency            = packet->payloadHeaderData.CenterFrequency;

    for(uint8_t i = 0; i < packet->payloadHeaderData.DetectionsInPacket; i++) {
        radar_driver::msg::RadarDetection data;

        //Filter through error flags in packet
        if (packet->resListDataArray[i].ui_Pdh0 & NEAR_PROB_MASK) {
            continue;
        } else if (packet->resListDataArray[i].ui_Pdh0 & INFERENCE_PROB_MASK) {
            continue;
        } else if (packet->resListDataArray[i].ui_Pdh0 & SIDELOBE_PROB_MASK) {
            continue;
        }

        data.pdh0          = packet->resListDataArray[i].ui_Pdh0;

        data.az_ang0       = packet->resListDataArray[i].f_AzAng0;
        data.rcs0          = packet->resListDataArray[i].f_RCS0;
        data.az_ang_var0   = packet->resListDataArray[i].f_AzAngVar0;
        data.prob0         = packet->resListDataArray[i].f_Prob0;

        data.az_ang1       = packet->resListDataArray[i].f_AzAng1;
        data.rcs1          = packet->resListDataArray[i].f_RCS1;
        data.az_ang_var1   = packet->resListDataArray[i].f_AzAngVar1;
        data.prob1         = packet->resListDataArray[i].f_Prob1;

        // Sanity check to make sure that assumption0 is indeed the better assumption
        //assert(data.Prob0 >= data.Prob1);

        data.vrel_rad     = packet->resListDataArray[i].f_VrelRad;
        //Told to ignore ElAng by continental, but store anyways since we parsed it
        data.el_ang       = packet->resListDataArray[i].f_ElAng;
        data.range_var    = packet->resListDataArray[i].f_RangeVar;
        data.vrel_rad_var = packet->resListDataArray[i].f_VrelRadVar;
        data.el_ang_var   = packet->resListDataArray[i].f_ElAngVar;
        data.snr          = packet->resListDataArray[i].f_SNR;
        data.range        = packet->resListDataArray[i].f_Range;

        // Updated calculations based on email thread with Conti
        // AzAng0 should always be the better of the two assupmtions
        data.pos_x = packet->resListDataArray[i].f_Range * cos(data.az_ang0);
        data.pos_y = packet->resListDataArray[i].f_Range * sin(data.az_ang0);
        data.pos_z = 0.0;  // Z really should not be in your interface.

        msg->detections.push_back(data);
    }
}

// Currently unused
void loadSSMessage(SSPacket_t* packet, radar_driver::msg::SensorStatus* msg) {
    msg->part_number             = packet->PartNumber;
    msg->assembly_part_number    = packet->AssemblyPartNumber;
    msg->sw_part_number          = packet->SWPartNumber;
    for (uint8_t i = 0; i < SENSOR_SERIAL_NUM_LEN; i++) {
        msg->serial_number[i]    = packet->SerialNumber[i];
    }
    msg->bl_version              = packet->BLVersion;
    msg->sw_version              = packet->SWVersion;
    msg->utc_time_stamp          = packet->UTCTimeStamp;
    msg->time_stamp              = packet->TimeStamp;
    msg->surface_damping         = packet->SurfaceDamping;
    msg->op_state                = packet->OpState;
    msg->current_far_cf          = packet->CurrentFarCF;
    msg->current_near_cf         = packet->CurrentNearCF;
    msg->defective               = packet->Defective;
    msg->bad_supply_volt         = packet->BadSupplyVolt;
    msg->bad_temp                = packet->BadTemp;
    msg->gm_missing              = packet->GmMissing;
    msg->tx_power_status         = packet->TxPowerStatus;
    msg->maximum_range_far       = packet->MaxRangeFar;
    msg->maximum_range_near      = packet->MaxRangeNear;
}
