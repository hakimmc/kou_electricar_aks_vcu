#include <WiFi.h>
#include <HTTPClient.h>

//---------------------------------------------------------------------
const char * ssid = "Poyrazwifi_Calgin"; //wifi
const char * password = "Ah487602";      //pass
String SCRIPT_ID = "AKfycbwPHgADCyh5aIyiaku_zNwz4JcQDqKH62RFQf-NAreZSIO_-CLCMwYxN6BT0cpL1l_w-A"; //google script id
//---------------------------------------------------------------------

int yon,hiz,sicaklik,b1v,b2v,b3v,b4v,b5v,b6v,b7v,b8v,b9v,b10v,b11v,b12v,b13v,b14v,b15v,b16v,b17v,b18v,b19v,b20v,b21v,b22v,b23v,b24v;
const int sendInterval = 100;

void write_to_random_data(){
  int x= random(100);
  yon = x;
  hiz=++x,sicaklik=++x,b1v=++x,b2v=++x,b3v=++x,b4v=++x,b5v=++x,b6v=++x,b7v=++x,b8v=++x,b9v=++x,b10v=++x,b11v=++x,b12v=++x,b13v=++x,b14v=++x,b15v=++x,b16v=++x,b17v=++x,b18v=++x,b19v=++x,b20v=++x,b21v=++x,b22v=++x,b23v=++x,b24v=++x;
}
/************************************************************************************
 * write to google sheet function:
 **********************************************************************************/
void write_to_google_sheet(String params) {
   HTTPClient http;
   String url="https://script.google.com/macros/s/"+SCRIPT_ID+"/exec?"+params;
    Serial.print(url);
    Serial.println("Google Sheet'e gönderiliyor.");
    //---------------------------------------------------------------------
    //start 
    http.begin(url.c_str());
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpCode = http.GET();  
    Serial.print("HTTP Status Code: ");
    Serial.println(httpCode);
    //---------------------------------------------------------------------
    //get response from google sheet
    String payload;
    if (httpCode > 0) {
        payload = http.getString();
        Serial.println("Payload: "+payload);     
    }
    //---------------------------------------------------------------------
    http.end();
}
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx



/************************************************************************************
 *  setup()
 **********************************************************************************/
 void setup() {
  //--------------------------------------------
  Serial.begin(9600);
  delay(10);
  //--------------------------------------------
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("WiFi'ye bağlaniyor.");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000); //1 sn delay
    Serial.print(".");
  }
  Serial.println("Bağlanti kuruldu");
  //--------------------------------------------
}
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx



/************************************************************************************
 *  loop()
 **********************************************************************************/
void loop() {
  write_to_random_data();
  String s_yon = String(yon);
  String s_hiz = String(hiz);
  String s_sicaklik = String(sicaklik);
  String s_b1v = String(b1v);
  String s_b2v = String(b2v);
  String s_b3v = String(b3v);
  String s_b4v = String(b4v);
  String s_b5v = String(b5v);
  String s_b6v = String(b6v);
  String s_b7v = String(b7v);
  String s_b8v = String(b8v);
  String s_b9v = String(b9v);
  String s_b10v = String(b10v);
  String s_b11v = String(b11v);
  String s_b12v = String(b12v);
  String s_b13v = String(b13v);
  String s_b14v = String(b14v);
  String s_b15v = String(b15v);
  String s_b16v = String(b16v);
  String s_b17v = String(b17v);
  String s_b18v = String(b18v);
  String s_b19v = String(b19v);
  String s_b20v = String(b20v);
  String s_b21v = String(b21v);
  String s_b22v = String(b22v);
  String s_b23v = String(b23v);
  String s_b24v = String(b24v);
  write_to_google_sheet("yon="+s_yon+"&hiz="+s_hiz+"&sicaklik="+s_sicaklik+"&b1v="+s_b1v+"&b2v="+s_b2v+"&b3v="+s_b3v+"&b4v="+s_b4v+"&b5v="+s_b5v+"&b6v="+s_b6v+"&b7v="+s_b7v+"&b8v="+s_b8v+"&b9v="+s_b9v+"&b10v="+s_b10v+"&b11v="+s_b11v+"&b12v="+s_b12v+"&b13v="+s_b13v+"&b14v="+s_b14v+"&b15v="+s_b15v+"&b16v="+s_b16v+"&b17v="+s_b17v+"&b18v="+s_b18v+"&b19v="+s_b19v+"&b20v="+s_b20v+"&b21v="+s_b21v+"&b22v="+s_b22v+"&b23v="+s_b23v+"&b24v="+s_b24v);
  //delay(sendInterval);

  //delay(sendInterval);
}
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

