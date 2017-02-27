struct status_out{
  unsigned short StrActual;
  unsigned short DrvActual;
  float Voltage;
  float CurrentS;
  float CurrentD;
};
typedef struct status_out status_out;

struct ctrl_var {
  int inputSteer;
  int inputDrive;
  int inputBreak;
  bool reverse;
};
typedef struct ctrl_var ctrl_var;
