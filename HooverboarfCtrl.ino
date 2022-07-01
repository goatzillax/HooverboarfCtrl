#include <Arduino.h>
//  not interrupt safe
#include <CircularBuffer.h>

#include <ESPAsyncWebServer.h>  //  pulls in AsyncTCP.h and WiFi.h
bool wifi_ap = false;

#define USE_WIIMOTE
#ifdef USE_WIIMOTE
#include <ESP32Wiimote.h>
ESP32Wiimote wiimote;
//  ugh need a macro or dict or something this is getting out of hand
uint8_t nc_js_x_min;
uint8_t nc_js_x_ctr;
uint8_t nc_js_x_max;
uint8_t nc_js_y_min;
uint8_t nc_js_y_ctr;
uint8_t nc_js_y_max;
uint8_t nc_js_expo;

ButtonState button;
NunchukState nunchuk;
#endif

#include <Preferences.h>
Preferences preferences;

bool set_prefs(void) {
  int ret=0;
#ifdef USE_WIIMOTE
  if (preferences.begin("hooverboarf", false)) {
    ret += preferences.clear();  //  just whack any old garbo
    ret += preferences.putUChar("nc_js_x_min", nc_js_x_min);
    ret += preferences.putUChar("nc_js_x_ctr", nc_js_x_ctr);
    ret += preferences.putUChar("nc_js_x_max", nc_js_x_max);
    ret += preferences.putUChar("nc_js_y_min", nc_js_y_min);
    ret += preferences.putUChar("nc_js_y_ctr", nc_js_y_ctr);
    ret += preferences.putUChar("nc_js_y_max", nc_js_y_max);
    ret += preferences.putUChar("nc_js_expo", nc_js_expo);
    preferences.end();
  }
  return (ret == 8);
#else
  return true;  // whatevs
#endif
}

void get_prefs(void) {
  if (preferences.begin("hooverboarf", true)) {
#ifdef USE_WIIMOTE
    nc_js_x_min = preferences.getUChar("nc_js_x_min", 30);
    nc_js_x_ctr = preferences.getUChar("nc_js_x_ctr", 131);
    nc_js_x_max = preferences.getUChar("nc_js_x_max", 230);
    nc_js_y_min = preferences.getUChar("nc_js_y_min", 30);
    nc_js_y_ctr = preferences.getUChar("nc_js_y_ctr", 131);
    nc_js_y_max = preferences.getUChar("nc_js_y_max", 230);
    nc_js_expo = preferences.getUChar("nc_js_expo", 50);
#if 0
    Serial.printf( "x center %d", nc_js_x_ctr);
    Serial.printf(" y center %d", nc_js_y_ctr);
    Serial.printf(" expo %d", nc_js_expo);
#endif
#endif
    preferences.end();
  }
  else {
    Serial.println("error opening preferences");
  }
}

//  hoverboard serial stats
unsigned long hover_csum_fails=0;
unsigned long hover_ser_rcvd=0;
unsigned long hover_ser_frame_rcvd=0;

struct serial_feedback_s {
	uint16_t	start = 0xabcd;
	int16_t		cmd1 = 0;
	int16_t		cmd2 = 0;
	int16_t		speed_r = 0;
	int16_t		speed_l = 0;
	int16_t		vbat = 0;  //  why is this signed?
	int16_t		temp_c = 0;
	uint16_t	led = 0;
	uint16_t	csum;
} hover_feedback;

AsyncWebServer server(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<body>
<h1>Weapon of Grass Destruction</h1>
<h2>Hooverboarf</h2>
Feedback bytes received:	%HOVER_SER_RCVD%<br>
Feedback frames received:	%HOVER_SER_FRAME_RCVD%<br>
Feedback csum fails:		%HOVER_CSUM_FAILS%<br>
<br>
Voltage: %VBAT%  Temperature:  %TEMP_C%

<h2>Nunchuk</h2>
<form action="/setprefs" method="get">
  X Min    <input type="number" value="%NC_JS_X_MIN%" name="nc_js_x_min" min="0" max="255">
  X Center <input type="number" value="%NC_JS_X_CTR%" name="nc_js_x_ctr" min="0" max="255">
  X Max    <input type="number" value="%NC_JS_X_MAX%" name="nc_js_x_max" min="0" max="255"><br><br>
  Y Min    <input type="number" value="%NC_JS_Y_MIN%" name="nc_js_y_min" min="0" max="255">
  Y Center <input type="number" value="%NC_JS_Y_CTR%" name="nc_js_y_ctr" min="0" max="255">
  Y Max    <input type="number" value="%NC_JS_Y_MAX%" name="nc_js_y_max" min="0" max="255"><br><br>
  Expo <input type="number" value="%NC_JS_EXPO%" name="nc_js_expo" min="0" max="100"><br><br>
  <input type="submit" value="Submit">
</form>
</body>
</html>
)rawliteral";

String index_html_processor(const String& var) {
	if (var == "HOVER_SER_RCVD") {
		return String(hover_ser_rcvd);
	}
	if (var == "HOVER_SER_FRAME_RCVD") {
		return String(hover_ser_frame_rcvd);
	}
	if (var == "HOVER_CSUM_FAILS") {
		return String(hover_csum_fails);
	}
	if (var == "VBAT") {
		return String(hover_feedback.vbat / 100);
	}
	if (var == "TEMP_C") {
		return String(hover_feedback.temp_c / 10);
	}
	if (var == "NC_JS_X_MIN") {
		return String(nc_js_x_min);
	}
	if (var == "NC_JS_X_CTR") {
		return String(nc_js_x_ctr);
	}
	if (var == "NC_JS_X_MAX") {
		return String(nc_js_x_max);
	}
	if (var == "NC_JS_Y_MIN") {
		return String(nc_js_y_min);
	}
	if (var == "NC_JS_Y_CTR") {
		return String(nc_js_y_ctr);
	}
	if (var == "NC_JS_Y_MAX") {
		return String(nc_js_y_max);
	}
	if (var == "NC_JS_EXPO") {
		return String(nc_js_expo);
	}
	return String();
}

void WebSetPrefs(AsyncWebServerRequest *request) {
	int params = request->params();

	for (int i=0; i<params; i++) {
		AsyncWebParameter *p = request->getParam(i);
		//Serial.printf("%s %d\n", p->name().c_str(), p->value().toInt());
#ifdef USE_WIIMOTE
		if (p->name() == "nc_js_x_min") {
			nc_js_x_min = p->value().toInt();
		}
		if (p->name() == "nc_js_x_ctr") {
			nc_js_x_ctr = p->value().toInt();
		}
		if (p->name() == "nc_js_x_max") {
			nc_js_x_max = p->value().toInt();
		}
		if (p->name() == "nc_js_y_min") {
			nc_js_y_min = p->value().toInt();
		}
		if (p->name() == "nc_js_y_ctr") {
			nc_js_y_ctr = p->value().toInt();
		}
		if (p->name() == "nc_js_y_max") {
			nc_js_y_max = p->value().toInt();
		}
		if (p->name() == "nc_js_expo") {
			nc_js_expo = p->value().toInt();
		}
#endif
	}
	request->send(200, "text/plain", set_prefs() ? "OK" : "ubad");
}

#define USE_CRSF
#ifdef USE_CRSF
#include "src/crsf/crsf.h"

#define CRSF_RXD1 19
#define CRSF_TXD1 18

#define RC_BUFFER_SIZE 25
uint8_t _rcs_buf[RC_BUFFER_SIZE] {};

uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_count{};

#define RC_HI  2000
#define RC_MID 1500
#define RC_LOW 1000
#define RC_DISARMED RC_LOW

//  TAER 1234 mapping on my TX
//  probably could enum these too
#define RC_CHAN_ARM   4
#define RC_CHAN_STEER 1
#define RC_CHAN_SPEED 2

//  so the lore on ELRS and Betaflight is that the arm channel must be on AUX1 and is sent every packet.
//  Betaflight requires 4 disarm packets before actually disarming.
//  I want to eventually have one arm for the mower and one for the hooverboarf, but the mower is more important so that's going on AUX1.

//  might like real packet stats but would have to dig into the crsf code.
unsigned long crsf_pkts=0;
#endif

#define USE_OLED
#ifdef USE_OLED
#include <Wire.h>
#include <LOLIN_I2C_BUTTON.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);
I2C_BUTTON oled_button(DEFAULT_I2C_BUTTON_ADDRESS); //I2C Address 0x31
unsigned long last_button=0;
#endif

enum input_type {
#ifdef USE_CRSF
	INPUT_CRSF,
#endif
#ifdef USE_WIIMOTE
	INPUT_WIIMOTE
#endif
} input_mode;

#define HOVER_RXD2 17
#define HOVER_TXD2 16

//  aight this file is getting kind of long, probably time to split everything up
struct serial_cmd_s {
  uint16_t start = 0xabcd;
  int16_t  steer = 0;   // -1000 to 1000
  int16_t  speed = 0;   // -1000 to 1000
  uint16_t csum;
} hover_cmd;

//  Honestly if we manage this correctly we only need one frame.
CircularBuffer<byte,9*2> feedback_circbuf;

enum feedback_states_e {
	FEEDBACK_SEARCH,
	FEEDBACK_FOUND
} feedback_state = FEEDBACK_SEARCH;

//  hooverboarf code apparently will timeout after 160ms of not-getting-cmds
#define HOVER_SER_PERIOD 40
unsigned long hover_ser_last=0;

#define DEBUG_PERIOD 500
unsigned long last_print=0;

#define FS_MS 1000
unsigned long last_good=0;
bool failsafe=true;
bool disarm_received=false;

#define MOWER_BALE_PIN 23  //  pull to GND to prime (turns on LEDs and pulls the power pin up to 5v), release to stop
#define MOWER_POWER_PIN 5  //  pulse to GND to start

enum states { MOWER_DISARMED, MOWER_ARMED } mower_state;
//  for independent hooverboarf arm there's not much sense in a mower_arm without hooverboarf arm...

void setup() {
  //  bro just throttle the output don't make it weird bro
  Serial.begin(115200);
  WiFi.mode(WIFI_OFF);

#ifdef ESPRESSIF_PULLED_THEIR_HEAD_OUTTA_THEIR_ASS
  //digitalWrite(MOWER_BALE_PIN, HIGH);
  //digitalWrite(MOWER_POWER_PIN, HIGH);
  //pinMode(MOWER_BALE_PIN,OUTPUT_OPEN_DRAIN);  // arduino output mode documentation also lacking.
  //pinMode(MOWER_POWER_PIN,OUTPUT_OPEN_DRAIN);  // arduino output mode documentation also lacking.
#endif

  get_prefs();

#ifdef USE_WIIMOTE
   btStart();  //  if we're going to brownout, do it early.
   input_mode = INPUT_WIIMOTE;  // this is the default?
   wiimote.init();
   //wiimote.addFilter(ACTION_IGNORE, FILTER_ACCEL);
#else
   btStop();
#endif

  set_fs();
  pinMode(MOWER_BALE_PIN,OUTPUT);
  pinMode(MOWER_POWER_PIN,OUTPUT);

#ifdef USE_CRSF
  Serial1.begin(420000, SERIAL_8N1, CRSF_RXD1, CRSF_TXD1);
  Serial1.setTimeout(100);
#endif
  Serial2.begin(115200, SERIAL_8N1, HOVER_RXD2, HOVER_TXD2);  // this would kind of cause problems with a Wemos D1, except the 8266 doesn't even have a UART2 in the first place.
  Serial2.setTimeout(100);

#ifdef USE_OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(0);
  display.setTextColor(WHITE);
  display.display();  //  hurr durrrrr
#endif

}

void dump_serial() {
  int i;
  int max = 5;
  char buf[64];

  sprintf(buf, "str %d spd %d fs %s fb %d", hover_cmd.steer, hover_cmd.speed, failsafe ? "*":"-", hover_ser_frame_rcvd);
  Serial.println(buf);
}

void disarm_mower() {
	mower_state = MOWER_DISARMED;
	digitalWrite(MOWER_BALE_PIN, LOW);
	digitalWrite(MOWER_POWER_PIN, LOW);
}

void arm_mower() {
	mower_state = MOWER_ARMED;
	digitalWrite(MOWER_BALE_PIN, HIGH);
	digitalWrite(MOWER_POWER_PIN, HIGH);
}

void set_fs() {
    failsafe=true;
    disarm_received=false;

    hover_cmd.steer = 0;
    hover_cmd.speed = 0;

    disarm_mower();
}

bool fuckoff = false;

//  alright seriously break this file up

//  Not totally convinced this is that much better than the other version currently
void hover_ser_recv() {
	if (Serial2.available()) {
		//  transfer everything into a circular buffer
		uint8_t uart_buffer[sizeof(hover_feedback)];
		int recvd = Serial2.readBytes(uart_buffer, feedback_circbuf.available());
		hover_ser_rcvd += recvd;
		for (int i=0; i<recvd; i++) {
			if (!feedback_circbuf.push(uart_buffer[i])) {
				Serial.println("ur a moran");
			}
		}
	}
search:
	//  search for header
	if (feedback_state == FEEDBACK_SEARCH) {
		while (feedback_circbuf.size() >= sizeof(hover_feedback.start)) {
			//  prevent a persistent off-by-one-byte situation by only shifting a byte at a time during search
			uint16_t header = feedback_circbuf.shift() | ((uint16_t) feedback_circbuf[0] << 8);
			if (header == 0xabcd) {
				feedback_circbuf.shift();  //  header confirmed, shift off the other byte
				feedback_state = FEEDBACK_FOUND;
			}
		}
	}

	//  process frame
	if (feedback_state == FEEDBACK_FOUND) {
		if (feedback_circbuf.size() >= (sizeof(hover_feedback)-sizeof(hover_feedback.start))) { //  already pulled the head off like a psychotic child ripping the heads off all his/her/its dolls
			//  should have enough to do an entire frame so...  doo eet.
			struct serial_feedback_s new_feedback;

			new_feedback.start = 0xabcd;  //  already cornfirmed this

			int i=0;  //  this is..  pretty much the part that I hate.
			new_feedback.cmd1 =	feedback_circbuf[i++] | (feedback_circbuf[i++] << 8);
			new_feedback.cmd2 =	feedback_circbuf[i++] | (feedback_circbuf[i++] << 8);
			new_feedback.speed_r =	feedback_circbuf[i++] | (feedback_circbuf[i++] << 8);
			new_feedback.speed_l =	feedback_circbuf[i++] | (feedback_circbuf[i++] << 8);
			new_feedback.vbat =	feedback_circbuf[i++] | (feedback_circbuf[i++] << 8);
			new_feedback.temp_c =	feedback_circbuf[i++] | (feedback_circbuf[i++] << 8);
			new_feedback.led =	feedback_circbuf[i++] | (feedback_circbuf[i++] << 8);
			new_feedback.csum =	feedback_circbuf[i++] | (feedback_circbuf[i++] << 8);

			uint16_t csum = (uint16_t) (new_feedback.start ^
					new_feedback.cmd1 ^
					new_feedback.cmd2 ^
					new_feedback.speed_r ^
					new_feedback.speed_l ^
					new_feedback.vbat ^
					new_feedback.temp_c ^
					new_feedback.led);

			if (csum == new_feedback.csum) {
				hover_ser_frame_rcvd++;
				// yeah whatever.
				memcpy(&hover_feedback, &new_feedback, sizeof(hover_feedback));
				// remove the rest of the frame from the circbuf
				for (i=0; i<(sizeof(hover_feedback)-sizeof(hover_feedback.start)); i++) {
					feedback_circbuf.shift();
				}
			}
			else {
				hover_csum_fails++;
			}

			feedback_state = FEEDBACK_SEARCH;

			if (feedback_circbuf.size() >= sizeof(hover_feedback)) {
				goto search;  //  bad programmer no donut
			}  //  this actually shouldn't happen unless we make a prebotemously sized circbuf

		}
	}
}

#ifdef USE_OLED
void dump_oled() {
	char buf[64];

	display.clearDisplay();
	display.setCursor(0, 0);

	switch (input_mode) {
#ifdef USE_WIIMOTE
		case(INPUT_WIIMOTE):
			if (failsafe) {
				sprintf(buf, "W");
			}
			else {
				sprintf(buf, "W%3d%s%s%3d", nunchuk.xStick, button & BUTTON_C ? "C" : "-", button & BUTTON_Z ? "Z" : "-", nunchuk.yStick);
			}
			display.println(buf);
			break;
#endif
#ifdef USE_CRSF
		case(INPUT_CRSF):
			if (failsafe) {
				sprintf(buf, "C");
			}
			else {
				sprintf(buf, "C%4d%4d", _raw_rc_values[RC_CHAN_STEER], _raw_rc_values[RC_CHAN_SPEED]);
			}
			display.println(buf);

			break;
#endif
	}

	sprintf(buf, "H%5d", hover_cmd.steer);
	display.println(buf);
	sprintf(buf, "H%5d", hover_cmd.speed);
	display.println(buf);
	sprintf(buf, "H%dC%dV", hover_feedback.temp_c/10, hover_feedback.vbat/100);
	display.println(buf);
	sprintf(buf, "M%4d", mower_state);
	//sprintf(buf, "H%d %d", hover_ser_rcvd, hover_csum_fails);
	display.println(buf);
	if (wifi_ap) {
		display.println(WiFi.softAPIP());
	}
	display.display();  // hurr durrrrrr
}
#endif

#ifdef USE_CRSF
void read_crsf(void) {
	//  oh right no way this can cause any starvation right?  right?
	while (Serial1.available()) {
		//  Not sure if I totally trust this code actually
		size_t numBytesRead = Serial1.readBytes(_rcs_buf, RC_BUFFER_SIZE);
		if(numBytesRead > 0) {
			if (crsf_parse(&_rcs_buf[0], numBytesRead, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS )) {
				last_good = millis();
				crsf_pkts++;  //  probably not really packets
				failsafe = false;

				hover_cmd.steer = constrain(map(_raw_rc_values[RC_CHAN_STEER], 1000, 2000, -1000, 1000),-1000, 1000);
				hover_cmd.speed = constrain(map(_raw_rc_values[RC_CHAN_SPEED], 1000, 2000, -1000, 1000),-1000, 1000);
				break;  //  I dunno this might be wack but as soon as we get a valid packet we should just go do other stuff.
			}
		}
	}

	if (_raw_rc_values[RC_CHAN_ARM] > RC_MID) {
		//  we must affirmatively receive *some* disarm packets before rearming the mower at least.
		//  reason:  hooverboarf powers off due to temperature.
		//  user still has mower armed on his TX switch.  user turns hooverboarf back on which turns the RX and ESP32 back on.  (This architecture may change.)
		//  first packets it receives are actually arm, which is *badwrong*, or just *badong*.
		if (disarm_received) {
			if (mower_state == MOWER_DISARMED) {
				//Serial.println("armed");
				arm_mower();
			}
		}  //  if we've never received a disarm, never arm.
	}
	else {
		//if (mower_state == MOWER_ARMED) {
		//  might do the betaflight disarm-debounce?
		disarm_received = true;
		//Serial.println("disarmed");
		disarm_mower();
		//}
	}

}
#endif

#ifdef USE_WIIMOTE

//  crude integer approximation of expo.
//  not doing one for CRSF -- let the TX handle that one.
long wii_expo(uint8_t value, uint8_t min, uint8_t center, uint8_t max, uint8_t expo) {
  long x;

  //  this is so ugly lol
  if (value >= center) {
    x = constrain(map(value, center, max, 0, 1000), 0, 1000);
  }
  else {
    x = constrain(map(value, min, center, -1000, 0), -1000, 0);
  }

  //  alrighty apply the expo
  //  y = (1-(100-k)/100) * (x/100)^3 + (x * (100-k)/100)
  //  fml can't track this in my head while keeping in range
  long a = x*x*x/100;         //  divide by 10000 once more
  long b = (100-expo)*a/100;
  long c = (a - b) / 10000;
  x = c + (x - x * expo/100);
  //constrain(x, -1000, 1000);  //  one last time just in case?
  return(x);
}

void read_wiimote(void) {
	wiimote.task();  // eh hate the way this is done but whatevs

	if (wiimote.available(true) > 0) {
		if (TinyWiimoteNunchukConnected()) {
			last_good = millis();
			failsafe = false;

			button  = wiimote.getButtonState();
			//  still something goofy going on here with disconnects but I am seriously tired of fighting this code    
			nunchuk = wiimote.getNunchukState();
#if 0
			char cc     = (button & BUTTON_C)     ? 'C' : '.';
			char cz     = (button & BUTTON_Z)     ? 'Z' : '.';
			Serial.print(cc);
			Serial.print(cz);
			Serial.printf(", nunchuk.stick: %3d/%3d\n", nunchuk.xStick, nunchuk.yStick);
#endif
			if (nunchuk.valid) {
				if (button & BUTTON_Z) {
					if (disarm_received) {
						if (mower_state == MOWER_DISARMED) {
							//Serial.println("armed");
							arm_mower();
						}
					}  //  if we've never received a disarm, never arm.
				}
				else {
					disarm_received = true;
					//Serial.println("disarmed");
					disarm_mower();
				}

				hover_cmd.steer = wii_expo(nunchuk.xStick, nc_js_x_min, nc_js_x_ctr, nc_js_x_max, nc_js_expo);
				hover_cmd.speed = wii_expo(nunchuk.yStick, nc_js_y_min, nc_js_y_ctr, nc_js_y_max, nc_js_expo);
			}
		}
		else {
			set_fs();
		}
	}

	if (!TinyWiimoteConnected()) {
		set_fs();
	}
}
#endif

void loop() {
	if (millis() - last_button > DEBUG_PERIOD*2) {
		last_button = millis();
		if (oled_button.get() == 0) {
			if (oled_button.BUTTON_A) {
				switch(oled_button.BUTTON_A) {
					case KEY_VALUE_SHORT_PRESS:
						Serial.println("change input");
						disarm_received=false;
						set_fs();
						switch (input_mode) {
#ifdef USE_WIIMOTE
							case INPUT_WIIMOTE:
								input_mode = INPUT_CRSF;
							break;
#endif
#ifdef USE_CRSF
							case INPUT_CRSF :
								input_mode = INPUT_WIIMOTE;
							break;
#endif
						}
						break;
					case KEY_VALUE_DOUBLE_PRESS:
						Serial.println("toggle wifi");
						if (wifi_ap) {
							Serial.println("This ish broke dawg just reset it");
							wifi_ap = false;
							WiFi.mode(WIFI_OFF);
							ESP.restart();
						}
						else {
							wifi_ap = true;
							fuckoff = true;
							btStop();
							WiFi.mode(WIFI_MODE_AP);
							WiFi.softAP("WGD");
							Serial.println(WiFi.softAPIP());

							server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
								request->send_P(200, "text/html", index_html, index_html_processor);
							});

							server.on("/setprefs", HTTP_GET, WebSetPrefs);

							server.begin();
						}
						break;
				}
			}
			if (oled_button.BUTTON_B) {
				//  uh, can't reach this button btw unless I poke de hole
				Serial.println("go take a dump");
				switch(oled_button.BUTTON_B) {
					case KEY_VALUE_SHORT_PRESS:
						break;
					case KEY_VALUE_DOUBLE_PRESS:
						break;
				}
			}
		}
	}

	if (fuckoff) {
		delay(100);
		return;
	}

	switch (input_mode) {
#ifdef USE_CRSF
		case(INPUT_CRSF):  read_crsf();  break;
#endif
#ifdef USE_WIIMOTE
		case(INPUT_WIIMOTE):  read_wiimote();  break;
#endif
	}

	if (millis()-last_good > FS_MS) {
		set_fs();
	}

	if (millis()-hover_ser_last > HOVER_SER_PERIOD) {
		hover_ser_last = millis();

		#define HOVER_SER_DEADBAND 10
		if ((hover_cmd.steer < HOVER_SER_DEADBAND) && (hover_cmd.steer > (-1*HOVER_SER_DEADBAND))) {
			hover_cmd.steer = 0;
		}

		if ((hover_cmd.speed < HOVER_SER_DEADBAND) && (hover_cmd.speed > (-1*HOVER_SER_DEADBAND))) {
			hover_cmd.speed = 0;
		}

		hover_cmd.csum = hover_cmd.start ^ hover_cmd.steer ^ hover_cmd.speed;

		//  I dunno kind of feels like there should be some infrastructure here to enforce endianness other than...  literally doing nothing at all.
		Serial2.write((uint8_t *) &hover_cmd, sizeof(hover_cmd));

	}  //  I LIKE WRITING MANUAL SCHEDULES IT IS FUN AND NOT AT ALL A BAD WAY TO TEACH PEOPLE HOW TO PROGRAM

	hover_ser_recv();

	if (millis()-last_print > DEBUG_PERIOD) {
		last_print = millis();
		dump_serial();
#ifdef USE_OLED
		dump_oled();
#endif
	}
}
