  //***********************************************************************
// Matlab .fis to arduino C converter v2.0.1.25122016                   
// - Karthik Nadig, USA                                                  
// Please report bugs to:                                                
// https://github.com/karthiknadig/ArduinoFIS/issues                     
// If you don't have a GitHub account mail to karthiknadig@gmail.com     
//***********************************************************************

#include "fis_header.h"
#include <AFMotor.h>
int trig = A0;
int echo = A1;

AF_DCMotor motor(3);

// Number of inputs to the fuzzy inference system
const int fis_gcI = 1;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 1;
// Number of rules to the fuzzy inference system
const int fis_gcR = 3;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

// Setup routine runs once when you press reset:
void setup()
{
    // initialize the Analog pins for input.
    // Pin mode for Input: water_level
    // pinMode(0 , INPUT);
    Serial.begin(9600);
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT); 


    // initialize the Analog pins for output.
    // Pin mode for Output: pump_rate
      //Set initial speed of the motor & stop
    motor.setSpeed(200);
    motor.run(RELEASE);
    // pinMode(1 , OUTPUT);

}


// Create Ultrasonic Function

int distance(int trig , int echo) {
 long t = 0, h = 0, hp = 0;
  
  // Transmitting pulse
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  // Waiting for pulse
  t = pulseIn(echo, HIGH);
  
  // Calculating distance 
  h = t / 58;
 
  h = h - 6;  // offset correction
  h = 50 - h;  // water height, 0 - 50 cm
  
  hp = 2 * h;
  return hp;
}

// Loop routine runs over and over again forever:
void loop()
{
    motor.run(FORWARD);
    // Read Input: water_level
    Serial.println(distance(trig, echo));
    g_fisInput[0] = distance(trig, echo); //analogRead(0);

    g_fisOutput[0] = 0;

    fis_evaluate();
    
    uint8_t ola = g_fisOutput[0];

    // Set output vlaue: pump_rate , CHECK LATER
    if (ola > 255){
      ola = 250;
    }
    Serial.print("pump speed : ");
    Serial.println(ola);
    motor.setSpeed(ola);  
    // analogWrite(1 , g_fisOutput[0]);
    delay(4000);

}



//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2*(b <= x)*(x <= c));
    if (b == c) return (FIS_TYPE) (t1*(a <= x)*(x <= b));
    t1 = min(t1, t2);
    return (FIS_TYPE) max(t1, 0);
}

FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b)
{
    return min(a, b);
}

FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b)
{
    return max(a, b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] =
{
    fis_trimf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 3 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 3 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { -43.2, 0, 43.2 };
FIS_TYPE fis_gMFI0Coeff2[] = { 10.8, 54, 97.2 };
FIS_TYPE fis_gMFI0Coeff3[] = { 64.8, 108, 151.2 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { -102, 0, 102 };
FIS_TYPE fis_gMFO0Coeff2[] = { 25.5, 127.5, 229.5 };
FIS_TYPE fis_gMFO0Coeff3[] = { 153, 255, 357 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 0, 0 };
int* fis_gMFI[] = { fis_gMFI0};

// Output membership function set
int fis_gMFO0[] = { 0, 0, 0 };
int* fis_gMFO[] = { fis_gMFO0};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1 };
int fis_gRI1[] = { 2 };
int fis_gRI2[] = { 3 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2 };

// Rule Outputs
int fis_gRO0[] = { 3 };
int fis_gRO1[] = { 2 };
int fis_gRO2[] = { 1 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 108 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 255 };

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System           
//***********************************************************************
FIS_TYPE fis_MF_out(FIS_TYPE** fuzzyRuleSet, FIS_TYPE x, int o)
{
    FIS_TYPE mfOut;
    int r;

    for (r = 0; r < fis_gcR; ++r)
    {
        int index = fis_gRO[r][o];
        if (index > 0)
        {
            index = index - 1;
            mfOut = (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else if (index < 0)
        {
            index = -index - 1;
            mfOut = 1 - (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else
        {
            mfOut = 0;
        }

        fuzzyRuleSet[0][r] = fis_min(mfOut, fuzzyRuleSet[1][r]);
    }
    return fis_array_operation(fuzzyRuleSet[0], fis_gcR, fis_max);
}

FIS_TYPE fis_defuzz_centroid(FIS_TYPE** fuzzyRuleSet, int o)
{
    FIS_TYPE step = (fis_gOMax[o] - fis_gOMin[o]) / (FIS_RESOLUSION - 1);
    FIS_TYPE area = 0;
    FIS_TYPE momentum = 0;
    FIS_TYPE dist, slice;
    int i;

    // calculate the area under the curve formed by the MF outputs
    for (i = 0; i < FIS_RESOLUSION; ++i){
        dist = fis_gOMin[o] + (step * i);
        slice = step * fis_MF_out(fuzzyRuleSet, dist, o);
        area += slice;
        momentum += slice*dist;
    }

    return ((area == 0) ? ((fis_gOMax[o] + fis_gOMin[o]) / 2) : (momentum / area));
}

//***********************************************************************
// Fuzzy Inference System                                                
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = FIS_MAX;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = FIS_MIN;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = fis_defuzz_centroid(fuzzyRuleSet, o);
        }
    }
}
