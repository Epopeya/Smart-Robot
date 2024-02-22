#include <WiFi.h>
#include <WebServer.h>
#include "secrets.h"

extern float current_direction;

WebServer server(80);

void handle_OnConnect() {
  Serial.println("New client connected");
  char* ptr = R"(<script>async function getData() {
  const response = await fetch("http://192.168.144.36/data", {mode: "no-cors"});
  const data = await response.json();
  console.log(data);
}
</script>)";

  server.send(200, "text/html", ptr);
}

void handle_dataRequest() {
  char data[1024];
  sprintf(data, "{\"dir\": %g}\n", current_direction);
  server.send(200, "application/json", data);
}

void setupServer() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Conectado a ");
  Serial.println(ssid);
  Serial.print("Direccion IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", handle_OnConnect);
  server.on("/data", handle_dataRequest);

  server.begin();
}

void loopServer() {
  server.handleClient();
}