
//Constant physical and electrical parameters
#define ANALOG_TO_ANGLE_GAIN 1.071819024
#define ANALOG_TO_ANGLE_OFFSET -2.772616596
#define ANALOG_TO_ANGLE_GAIN_I2C 1
#define ANALOG_TO_ANGLE_OFFSET_I2C 0
#define BELT_LENGTH_MM 2630
#define BELT_HEIGHT_MM 185
#define SPEED_TO_ANALOG_GAIN 1.03
#define SPEED_TO_ANALOG_OFFSET 0
#define PCT_TRESHOLD 12
#define ADC_I2C_ADDRESS 0x48
#define THRESHOLD_ANGLE_DEG 78
#define SLOPE_MM_PER_DEG 6.3853

//define a class to manage the lift of the custom project le mur
class Lift {
    float inclinaison_deg;
    float height_mm;
    public :
        Lift(char upPin, char downPin, char ADC_ADR, char DAC_ADR);
        ~Lift();    
        void update();
        float getInclinaison_deg();
        float getSensorValue(bool raw = true);
        float getHeight_mm();
        void setSpeed(float speed_pct);
        void moveUp();
        void moveDown();
        void stop();
        void move(float pid_output);
        float computeHeight(float angle_deg);
    private :
        //dac output form incinometer
        float sensorValue;
        int sensorValueRaw;
        //pins
        char inclinometerPin;
        bool i2c;
        char upPin;
        char downPin;
        char speedPin;
        //Fonction to convert angle to height
        float horizontalPosition(float angle_deg);
};