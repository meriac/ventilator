/* Display dependencies */
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
/* Servo dependencies*/
#include <Servo.h>

#define LOWPASS_FILTER 8
#define POTENTIOMETER_TRESHOLD 8

/* medical configuration */
#define BREATHING_RATE_MIN 10
#define BREATHING_RATE_MAX 30
#define BREATH_TIME_SLOPE_IN 570;
#define BREATH_TIME_SLOPE_OUT 380;

/* pin definitions */
#define PIN_POTI_RATE 0
#define PIN_POTI_VOLUME 1
#define PIN_SERVO 9

/* screen configuration */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

/* servo configuration */
#define SERVO_PULSE_MS_MIN 1120
#define SERVO_PULSE_MS_MAX 1030

/* Global Objects */
Servo g_servo;
Adafruit_SSD1306 g_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/* Potentiometer filtering */
static int g_poti_volume, g_poti_volume_raw;
static int g_poti_volume_lp[LOWPASS_FILTER];

static int g_poti_rate, g_poti_rate_raw;
static int g_poti_rate_lp[LOWPASS_FILTER];

static int g_pos_lp;

static int g_breathing_rate, g_breathing_volume;

/* breathing_rate is breaths per minute */
void do_breathe(bool force_reset)
{
  static byte state_machine;
  static unsigned long time_prev;
  static int angle_src, angle_dst, duration;

  unsigned long delta_t;
  int angle, pulse_max;

  /* enforce state machine reset */
  if(force_reset)
  {
    state_machine = 0xFF;
    time_prev = 0;
    duration = 0;
  }

  /* calculate elapsed time in this state */
  delta_t = millis() - time_prev;

  /* switch to next state if needed */
  if(delta_t>=duration)
  {
    delta_t = 0;
    time_prev = millis();

    /* switch to next state */
    state_machine++;
    if(state_machine > 3)
      state_machine = 0;

    /* calculate breathing amplitude - modulate angle 50%-100% */
    pulse_max = SERVO_PULSE_MS_MIN+((100+g_breathing_volume)*((long)(SERVO_PULSE_MS_MAX-SERVO_PULSE_MS_MIN))/200);

    /* obtain per-state parameters */
    switch(state_machine)
    {
      default:
      case 0:
        angle_src = SERVO_PULSE_MS_MIN;
        angle_dst = pulse_max;
        duration = BREATH_TIME_SLOPE_IN;
        break;
      case 1:
        angle_src = pulse_max;
        angle_dst = pulse_max;
        /* inhale 1/3rds of total breating time */
        duration = (((1UL*60*1000)/3)/g_breathing_rate)-BREATH_TIME_SLOPE_IN;
        break;
      case 2:
        angle_src = pulse_max;
        angle_dst = SERVO_PULSE_MS_MIN;
        duration = BREATH_TIME_SLOPE_OUT;
        break;
      case 3:
        angle_src = SERVO_PULSE_MS_MIN;
        angle_dst = SERVO_PULSE_MS_MIN;
        /* exhale 2/3rds of total breating time */
        duration = (((2UL*60*1000)/3)/g_breathing_rate)-BREATH_TIME_SLOPE_OUT;
        break;
    }
  }

  angle = angle_src + (((angle_dst-angle_src)*(long)delta_t)/duration);

  g_servo.writeMicroseconds(angle);
}

bool updated_potentiometers(bool force_update)
{
  int val;
  bool changed;

  changed = false;
 
  /* read breathing rate potentiometer */
  val = analogRead(PIN_POTI_RATE);
  g_poti_rate_raw += val - g_poti_rate_lp[g_pos_lp];
  g_poti_rate_lp[g_pos_lp] = val;
  val = g_poti_rate_raw/LOWPASS_FILTER;
  /* check if changes are above treshold */
  if(force_update || (abs(g_poti_rate-val)>=POTENTIOMETER_TRESHOLD))
  {
    g_poti_rate = val;
    /* map breathing rate potentiometer to breathing rate */
    val = map(1023-g_poti_rate,0,1023,BREATHING_RATE_MIN-2,BREATHING_RATE_MAX+2);
    g_breathing_rate = val>BREATHING_RATE_MAX ? BREATHING_RATE_MAX : (val<BREATHING_RATE_MIN ? BREATHING_RATE_MIN : val);
    changed = true;
  }
  
  /* read tidal volume potentiometer */
  val = analogRead(PIN_POTI_VOLUME);
  g_poti_volume_raw += val - g_poti_volume_lp[g_pos_lp];
  g_poti_volume_lp[g_pos_lp] = val;
  val = g_poti_volume_raw/LOWPASS_FILTER;
  /* check if changes are above treshold */
  if(force_update || (abs(g_poti_volume-val)>=POTENTIOMETER_TRESHOLD))
  {
    g_poti_volume = val;
    /* map breathing volume potentiometer to breathing volume */
    val = map(1023-g_poti_volume,0,1023,-2,102);
    g_breathing_volume = val>100 ? 100 : (val<0 ? 0 : val);
    changed = true;
  }

  /* wrap around ind lowpass buffer */
  g_pos_lp++;
  if(g_pos_lp>=LOWPASS_FILTER)
    g_pos_lp=0;  

  return changed;
}

void update_display(void)
{
  g_display.clearDisplay();
  g_display.setCursor(0,0);
  g_display.print("\nRate  :"); g_display.println(g_breathing_rate, DEC);
  g_display.print("Volume:"); g_display.println(g_breathing_volume, DEC);
  g_display.display();  
}

void setup() {
  int i;

  /* Serial Port configuration */
  Serial.begin(115200);

  /* Initialize potentiometer lowpass filters */
  g_pos_lp=0;
  memset(g_poti_volume_lp,0,sizeof(g_poti_volume_lp));
  memset(g_poti_rate_lp,0,sizeof(g_poti_rate_lp));
  /* Initilaize potentiometer inputs */
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  /* Fill lowpass filter with ADC values */
  for(i=0; i<LOWPASS_FILTER; i++)
    updated_potentiometers(true);
  
  /* Setup servo at default outer position */
  g_servo.attach(PIN_SERVO);
  /* reset breathing state machine */
  do_breathe(true);

  /* Initialize Display */
  if(!g_display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while(1)
      Serial.println(F("ERROR: SSD1306 allocation failed"));
  }
  g_display.setTextSize(2);
  g_display.setTextColor(SSD1306_WHITE);
  g_display.cp437(true);
  update_display();
}

void loop() {
  /* read potentiometers */
  if(updated_potentiometers(false))
    update_display();
 
  /* update breathing cycle */
  do_breathe(false);
 }
