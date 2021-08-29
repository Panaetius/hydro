

void sendJsonReponse(WiFiEspClient client){
  client.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: application/json\r\n"
    "Connection: close\r\n"  // the connection will be closed after completion of the response
    "\r\n");
#if use_temp
  temp_json_response(client);
#endif
#if use_ph
  ph_json_response(client);
#endif
#if use_ec
  ec_json_response(client);
#endif
#if use_dht
  dht_json_response(client);
#endif
#if use_fan
  fan_json_response(client);
#endif
#if use_foggers
  foggers_json_response(client);
#endif
#if use_pumps
  pumps_json_response(client);
#endif
#if use_hpa_pump
  hpa_pump_json_response(client);
#endif
  client.print("}\r\n");
}

void sendHttpResponse(WiFiEspClient client, float waterTemp, float ecValue, float phValue){
  // send a standard http response header
  // use \r\n instead of many println statements to speedup data send
  client.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Connection: close\r\n"  // the connection will be closed after completion of the response
    "\r\n");
  client.print("<!DOCTYPE HTML>\r\n");
  client.print("<html>\r\n");
  client.print("<h1>Hydroponic Control!</h1>\r\n");
  client.print("<br><b>Water Temperature: </b>");
  client.print(waterTemp);
  client.print("&deg;C<br><b>PH: </b>");
  client.print(phValue);
  client.print("<br><b>EC: </b>");
  client.print(ecValue);
  client.print("&micro;S<br><b>Fan RPM: </b>");
  client.print(rpm);
  client.print("<br><br><form >\r\n");
  client.print("<b>Fan Speed: </b>0<input type='range' min='0' max='100' value='");
  client.print(fanSpeed);
  client.print("' class='slider' name='spd'>100(");
  client.print(fanSpeed);
  client.print(")<br><br>\r\n");
  client.print("<input type='submit' name='Submit'></form>\r\n");
  client.print("</html>\r\n");
}
