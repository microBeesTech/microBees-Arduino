
void setup(){
Serial.begin(115200);
}
WiFiClient client;
void doPost(){

const char* host="https://dev.microbees.com/v/1_0/sendUpdate";
String PostData = "";

if (client.connect(host, 80)) {

client.println("POST /posts HTTP/1.1");
client.println("Host: dev.microbees.com.com");
client.println("Authorization: bearer {YOUR-USER-TOKEN}");
client.println("Content-Type: application/json");
client.print("Content-Length: ");
client.println(PostData.length());
client.println();
client.println(PostData);

long interval = 2000;
unsigned long currentMillis = millis(), previousMillis = millis();

while(!client.available()){

  if( (currentMillis - previousMillis) > interval ){

    Serial.println("Timeout");
    client.stop();     
    return;
  }
  currentMillis = millis();
}
}
