#include <Wire.h>
#include "Lp5569.h"
Lp5569Engines ledChip(0x32);
static uint32_t next;

static const uint16_t program[] =
{
  // Engine one
  0x9c10, // 0 map start
  0x9c9f, // 1 map end
};


void setup() {
  Serial.begin(9600);
  delay(5000);
  Serial.println("-- Starting Setup() --");

  ledChip.Begin();
  ledChip.Enable();

  delay(500);
  
  ledChip.ClearInterrupt();
  Serial.println(sizeof(program));
  if (ledChip.LoadProgram(program, (sizeof(program) / 2)))
  {
    Serial.println("Program loaded?");

    if (ledChip.VerifyProgram(program, (sizeof(program) / 2)))
    {
      Serial.println("program verifies");
    }
  }
  else
  {
    Serial.println("Program didn't load?");
    Serial.println(ledChip.LP_ERR_NONE);
  }

  next = millis() + 3000;
 ledChip.ClearInterrupt();

  ledChip.SetEngineEntryPoint(1, 0);
  ledChip.Womp();
  ledChip.SetEnginePC(1, 0);

  ledChip.Womp();
  ledChip.SetEngineModeFree(1);
  //ledChip.SetEngineModeFree(1);
  //ledChip.setEngineModeStep(2);
  //ledChip.SetEngineModeFree(2);

  ledChip.SetEngineRunning(1);
  //ledChip.SetEngineRunning(1);
  //ledChip.SetEngineRunning(2);

  Serial.println("### Setup complete");
}

void loop() 
{
 int32_t result;
  int8_t  val;
  static uint32_t count = 0;

  if(millis() >= next)
  {
    next += 1000;
    count++;
/*
    Serial.print("#");
    Serial.println(count);

    Serial.print(ledChip.GetEnginePC(1));
    Serial.print("");
    Serial.print(" ");
    Serial.print(ledChip.GetEngineMode(1));
    Serial.print("\n");*/
  } 

}
