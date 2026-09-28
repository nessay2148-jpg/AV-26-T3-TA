// Part A: This is an extension task that requires you to decode sensor data from CAN log files.
// CAN (Controller Area Network) is a communication standard used in automotive applications (including Redback cars)
// to allow communication between sensors and controllers.
//
// Your Task: Using the signal definitions in SteeringBench.dbc, read each CAN capture in data/
// and turn it into a CSV with one row per decoded frame:
// t,u_commanded,y_measured
// eg:
// 0,15.0,0.0
// 0.005,15.0,0.0
// ...
// where t is the frame timestamp minus the first kept frame's timestamp (s), u_commanded is
// the decoded CmdAngularRate (deg/s), and y_measured is the decoded MeasuredAngle (deg).
// The above values are not real numbers; they are only there to show the expected data output format.
// Do this for all three captures:
// data/step_test.log       ->  data/step_test.csv
// data/reversal_test.log   ->  data/reversal_test.csv
// data/deadband_test.log   ->  data/deadband_test.csv
//
// The Row type, writeCsv(), and main() below are provided -- they loop the three logs, call your
// decodeLog(), and write the CSV in exactly the format above. You just need to implement decodeLog().
//
// You do not need to use any external libraries. Use the resources below to understand how to
// extract sensor data.
// Hint: Think about manual bit masking and shifting, data types required,
// what formats are used to represent values, etc.
// Resources:
// https://www.csselectronics.com/pages/can-bus-simple-intro-tutorial
// https://www.csselectronics.com/pages/can-dbc-file-database-intro
//
// Sanity check: plot your CSVs (python3 plot_data.py) and compare against the pre-plotted
// data/*.png files -- they should match.
//
// Build & run (from the TA/ folder):
//     c++ -std=c++17 Question-A.cc -o decode
//     ./decode

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

// One output row.
struct Row {
    double t;            // seconds since the first kept frame
    double u_commanded;  // deg/s
    double y_measured;   // deg
};

// Read the candump log at `path` and return one Row per STEER_ActuatorLog frame, in order.
// Push one Row{t, u_commanded, y_measured} per kept frame.
std::vector<Row> decodeLog(const std::string& path) {
    std::vector<Row> rows;
    // brian storm: isolate timestamp, extract only the relevant data (first 8 numbers) in the hex log and 
    // convert into bytes. first 2 bytes should correspond to y_measured and next 2 for u_commanded
    // first open the file and read in the lines individually until there aren't any left
    // using a while loop
    std::ifstream file(path);
    std::string line;

    bool has_t0 = false;
    double t0 = 0.0;

    while (std::getline(file, line)) {
        // find the timestamps: are found between the brackets
        size_t start = line.find('(');
        size_t end = line.find(')');
        std::string tsString = line.substr(start + 1, end - start - 1); // isolate timestamp
        double timestamp = std::stod(tsString);
        // sets the very first timestamp to t0
        if (!has_t0) {
            t0 = timestamp;
            has_t0 = true;
        }
        // filter out the logs that don't start with 0x200, split at the #
        // sets id_string to the string found after the space after vcan0 and before the #
        size_t hashtag_position = line.find('#');
        std::string before_hashtag = line.substr(0, hashtag_position);
        size_t last_space = before_hashtag.find_last_of(' ');
        std::string id_string = before_hashtag.substr(last_space + 1);
        // check if id_string is 200. chuck out anything that isnt 200
        int id = std::stoi(id_string, nullptr, 16);
        if (id != 0x200) {
            // preserves the first timestamp t0
            continue;
        }
        // the value of t of every log is equal to its timestamp minus t0
        double t = timestamp - t0;
        
        std::string payload = line.substr(hashtag_position + 1);
        // string that contains the 4 bytes that we actually want (from the payload string)
        // select 2 characters from the payload substring at a time, each loop jumps to 2 characters after
        // must unsign the bytes when extracting raw data
        unsigned char bytes[4];
        for (int i = 0; i < 4; i++) {
            std::string byte_string = payload.substr(i * 2, 2);
            bytes[i] = static_cast<unsigned char>(std::stoi(byte_string, nullptr, 16));
        }
        // measured angle in degrees, scale by 0.1. uses bytes 0 and 1
        // little endian says that the first byte is smaller than the second
        // build the raw 16 bits and store in an unsigned 16 bit variable 
        // then reinterpret as an actual signed value
        // the dbc file requires the signal to be signed (-) as steering angle and angular rate
        // are physically things that go in the negative direction. therefore final value is signed
        int raw_measured_angle = bytes[0] + bytes[1] * 256;
        int signed_angle; 
        if (raw_measured_angle >= 32768) {
            signed_angle = raw_measured_angle - 65536;
        }
        else {
            signed_angle = raw_measured_angle;
        }
        double measured_angle = 0.1 * signed_angle;
        // CmdAngular rate in deg/sec, scale by 0.1. uses bytes 2 and 3
        // same structure of calculating values as above

        int raw_angular_rate = bytes[2] + bytes[3] * 256;
        int signed_rate; 
        if (raw_angular_rate >= 32768) {
            signed_rate = raw_angular_rate - 65536;
        }
        else {
            signed_rate = raw_angular_rate;
        }
        double cmd_angular_rate = 0.1 * signed_rate;

        // store each row vector and their values 
        Row r;
        r.t = t;
        r.u_commanded = cmd_angular_rate;
        r.y_measured = measured_angle;
        rows.push_back(r);
    }

    return rows;
}

// Provided -- writes the rows to a CSV in the required format. Do not change.
void writeCsv(const std::string& path, const std::vector<Row>& rows) {
    std::ofstream f(path);
    f << "t,u_commanded,y_measured\n";
    for (const Row& r : rows)
        f << r.t << "," << r.u_commanded << "," << r.y_measured << "\n";
}

// Provided -- runs decodeLog() + writeCsv() for each of the three captures.
int main() {
    const char* names[] = {"step_test", "reversal_test", "deadband_test"};
    for (const char* n : names) {
        const std::string in  = std::string("data/") + n + ".log";
        const std::string out = std::string("data/") + n + ".csv";
        const std::vector<Row> rows = decodeLog(in);
        writeCsv(out, rows);
        std::printf("%-14s %6zu frames -> %s\n", n, rows.size(), out.c_str());
    }
    return 0;
}
