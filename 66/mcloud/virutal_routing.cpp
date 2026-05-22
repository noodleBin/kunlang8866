#include "convert.h"
#include "mcloud.h"
#include "convert.h"
#include <iostream>
#include <cstdlib>
#include <iomanip>
#include <vector>
#include <string>
#include "vehicle_message.h"
#include <sstream>
#include "nlohmann/json.hpp"
int main() {

  double offset_x_ = {250932.85};
  double offset_y_ = {3987498.59};
    double utm_x = offset_x_ +  -2482.86;
    double utm_y = offset_y_ + 2209.42;
    char zone[20] = "51S";
    double lon, lat;
    std::cout << std::fixed << std::setprecision(10);
    UtmtoWgs84(utm_x, utm_y, zone, &lon, &lat, nullptr, nullptr);
    // UtmtoWgs84();
    // Wgs84toUtm(longitude * DEG_TO_RAD, latitude * DEG_TO_RAD, &utm_x, &utm_y,
    //         nullptr, nullptr, zone);
    std::cout << lon * RAD_TO_DEG << " " << lat * RAD_TO_DEG << std::endl;
    return 0;
}

// #include "mcloud.h"
// #include "convert.h"
// #include <iostream>
// #include <cstdlib>
// #include <iomanip>
// #include <vector>
// #include <string>
// #include "vehicle_message.h"
// #include <sstream>
// #include "nlohmann/json.hpp"

// using json = nlohmann::json;

// json parseINSPVAXA(const std::string& data) {
//     std::vector<std::string> fields;
//     std::istringstream stream(data);
//     std::string token;

//     for (char ch : data) {
//         if (ch == ',' || ch == ';') {
//             if (!token.empty()) {
//                 fields.push_back(token);
//                 token.clear();
//             }
//         } else {
//             token += ch;
//         }
//     }
    
//     for (auto field : fields) {
//       std::cout << field << " ";
//     }
//     std::cout << std::endl;
//     std::cout << "fields : " << fields.size() << std::endl;
//     json j;
//     try {
//         j["Header"] = fields.at(0);
//         j["Device ID"] = fields.at(1);
//         j["Message Counter"] = std::stoi(fields.at(2));
//         j["Timestamp"] = std::stod(fields.at(3));
//         j["Clock Mode"] = fields.at(4);
//         j["GPS Week"] = std::stoi(fields.at(5));
//         j["GPS Seconds"] = std::stod(fields.at(6));
//         j["Reference Station ID"] = fields.at(7);
//         j["INS Status"] = fields.at(8);
//         j["Position Type"] = fields.at(9);
//         j["RTK Status"] = fields.at(10);
//         j["Latitude"] = std::stod(fields.at(11));
//         j["Longitude"] = std::stod(fields.at(12));
//         j["Altitude"] = std::stod(fields.at(13));
//         j["Undulation"] = std::stof(fields.at(14));
//         j["North Velocity"] = std::stod(fields.at(15));
//         j["East Velocity"] = std::stod(fields.at(16));
//         j["Up Velocity"] = std::stod(fields.at(17));
//         j["Roll"] = std::stod(fields.at(18));
//         j["Pitch"] = std::stod(fields.at(19));
//         j["Azimuth"] = std::stod(fields.at(20));
//         j["Latitude Sigma"] = std::stof(fields.at(21));
//         j["Longitude Sigma"] = std::stof(fields.at(22));
//         j["Altitude Sigma"] = std::stof(fields.at(23));
//         j["North Velocity Sigma"] = std::stof(fields.at(24));
//         j["East Velocity Sigma"] = std::stof(fields.at(25));
//         j["Up Velocity Sigma"] = std::stof(fields.at(26));
//         j["Roll Sigma"] = std::stof(fields.at(27));
//         j["Pitch Sigma"] = std::stof(fields.at(28));
//         j["Azimuth Sigma"] = std::stof(fields.at(29));
//         j["Extended Solution Status"] = fields.at(30);
//         j["Time Since Update"] = std::stoi(fields.at(31));

//         std::string crcField = fields.at(32);
//         if (crcField[0] == '*') {
//             crcField = crcField.substr(1);
//         }
//         j["CRC"] = crcField;

//     } catch (std::out_of_range& e) {
//         std::cerr << "Error parsing data: " << e.what() << std::endl;
//     }

//     return j;
// }

// std::vector<uint8_t> jsonToUint8Array(const json& parsed_json) {
//     std::string json_string = parsed_json.dump();
//     return std::vector<uint8_t>(json_string.begin(), json_string.end());
// }

// int main() {
//     std::string data = "#INSPVAXA,ICOM4,0,0.0,FINESTEERING,2107,35489.000,00000000,03de,68;INS_ALIGNMENT_COMPLETE,INS_RTKFIXED,28.23316396165,112.87713086609,82.7966,-17.0382,0.0020,-0.0191,0.0006,179.789714292,-0.387541550,1.405962922,0.0240,0.0168,0.0218,0.0047,0.0049,0.0054,0.0553,0.0553,1.0818,00000000,0*fd6e3a89";

//     json parsed_json = parseINSPVAXA(data);

//     std::cout << parsed_json.dump(4) << std::endl;

//     std::vector<uint8_t> data_uint = jsonToUint8Array(parsed_json);

//     century::mcloud::VehicleMessage reply(0xFE, 0XEE, "LK1200009PS000000", 0x01, data_uint);

//     auto msg = reply.pack_message();

//     for (uint8_t byte : msg) {
//         std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
//     }
//     std::cout << std::endl;

//     for (uint8_t byte : reply.data_unit) {
//         std::cout << byte;
//     }
//     std::cout << std::endl;

//     return 0;
// }
