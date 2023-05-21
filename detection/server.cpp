#include "server.h"

void handleRoot() {
  String html = "<html><head>"
                "<style>"
                "body {"
                "  font-family: Arial, sans-serif;"
                "  background-color: #f2f2f2;"
                "}"
                "h1 {"
                "  color: #333333;"
                "  text-align: center;"
                "}"
                "#activity {"
                "  font-size: 24px;"
                "  font-weight: bold;"
                "  display: block;"
                "}"
                "</style>"
                "<script>"
                "function getActivity() {"
                "  var xhttp = new XMLHttpRequest();"
                "  xhttp.onreadystatechange = function() {"
                "    if (this.readyState == 4 && this.status == 200) {"
                "      document.getElementById('activity').innerText = this.responseText;"
                "    }"
                "  };"
                "  xhttp.open('GET', '/activity', true);"
                "  xhttp.send();"
                "}"
                "setInterval(getActivity, 1000);"  // get activity every 1 second
                "</script></head><body>"
                "<h1>Activity: <span id=\"activity\"></span></h1>"
                "</body></html>";
  server.send(200, "text/html", html);
}

void handleActivity() {
  server.send(200, "text/plain", activity);
}