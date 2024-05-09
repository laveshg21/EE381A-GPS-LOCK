#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <math.h>

// Define software serial ports for Bluetooth and GPS
SoftwareSerial bluetoothSerial(12, 11);
SoftwareSerial gpsSerial(3, 4);

// Define constants
const int relayPin = 2;
const double RADIUS_EARTH = 6371000;
const double threshold = 8.0;

// Function to convert degrees to radians
double toRadians(double degree) {
    return degree * M_PI / 180.0;
}

// Function to calculate distance between two GPS coordinates using Haversine formula
double haversine(double lat1, double lon1, double lat2, double lon2) {
    lat1 = toRadians(lat1);
    lon1 = toRadians(lon1);
    lat2 = toRadians(lat2);
    lon2 = toRadians(lon2);

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
    double c = 2 * asin(sqrt(a));
    double d = RADIUS_EARTH * c;

    return d;
}

// Setup function
void setup() {
    Serial.begin(9600);
    bluetoothSerial.begin(9600);
    pinMode(relayPin, OUTPUT);
}

// Loop function
void loop() {
    if (bluetoothSerial.available()) {
        while (gpsSerial.available() > 0) {
            if (gpsSerial.find("$GPRMC")) {
                float latitude = parseDegree();
                char lat = gpsSerial.read();
                if (lat == 'S') {
                    latitude = -latitude;
                }
                float longitude = parseDegree();
                char lon = gpsSerial.read();
                if (lon == 'W') {
                    longitude = -longitude;
                }
            }
        }
        String data = bluetoothSerial.readString();
        if (data.length() == 1) {
            char receivedChar = data.charAt(0);
            if (receivedChar == '1') {
                unlockDoor();
                delay(1000);
            } else if (receivedChar == '0') {
                lockDoor();
                delay(1000);
            }
            delay(1000);
        } else {
            int commaIndex = data.indexOf(',');
            if (commaIndex >= 0) {
                String latitudeStr = data.substring(0, commaIndex);
                String longitudeStr = data.substring(commaIndex + 1);
                double latitude = latitudeStr.toDouble();
                double longitude = longitudeStr.toDouble();
                Serial.print(latitude, 6);
                Serial.print(",");
                Serial.println(longitude, 6);
                double distance = haversine(lat1, lon1, latitude, longitude);
                if (distance > threshold) {
                    lockDoor();
                }
            }
        }
    }
}

// Function to unlock the door
void unlockDoor() {
    digitalWrite(relayPin, LOW);
}

// Function to lock the door
void lockDoor() {
    digitalWrite(relayPin, HIGH);
}

// Function to parse GPS degree format to decimal
float parseDegree() {
    float degree = gpsSerial.parseFloat();
    float minute = gpsSerial.parseFloat();
    return degree + minute / 60.0;
}
