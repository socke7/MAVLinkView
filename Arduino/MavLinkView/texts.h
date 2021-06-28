#ifndef _TEXTS_H_

#define _TEXTS_H_


#define LabelRowCount 8
#define LabelColumnCount 2
const char * boxLabel[LabelRowCount][LabelColumnCount] =
{
  { "POS", NULL  },
  { "ALT", "RAL" },
  { "ASP", "GSP" },
  { "HDG", "CLB" },
  { "THR", "MOD" },
  { "VOL", "CUR" },
  { "SAT", "HDP" },
  { "FIX", "HOM" }
};

#define GpsFixTextCount 9
const char * GpsFixText[GpsFixTextCount] =
{
  "no GPS",
  "no fix",
  "2D",
  "3D",
  "DGPS",
  "RTK float",
  "RTK fixed",
  "static",
  "ppp"
};

#define PlaneModeTextCount 25
const char * PlaneModeText[PlaneModeTextCount] =
{
  "MANUAL",
  "CIRCLE",
  "STABILIZE",
  "TRAINING",
  "ACRO",
  "FBWA",
  "FBWB",
  "CRUISE",
  "AUTOTUNE",
  "XXX",
  "AUTO",
  "RTL",
  "LOITER",
  "TAKEOFF",
  "AVOID_ADSB",
  "GUIDED",
  "INIT",
  "QSTAB",
  "QHOVER",
  "QLOITER",
  "QLAND",
  "QRTL",
  "QAUTOTUNE",
  "QACRO",
  "THERMAL"
};

#define CopterModeTextCount 27
const char * CopterModeText[CopterModeTextCount] =
{
  "STABILIZE",
  "ACRO",
  "ALT_HOLD",
  "AUTO",
  "GUIDED",
  "LOITER",
  "RTL",
  "CIRCLE",
  "XXX",
  "LAND",
  "XXX",
  "DRIFT",
  "XXX",
  "SPORT",
  "FLIP",
  "AUTOTUNE",
  "POSHOLD",
  "BRAKE",
  "THROW",
  "AVOID_ADSB",
  "GDED_NOGPS",
  "SMART_RTL",
  "FLOWHOLD",
  "FOLLOW",
  "ZIGZAG",
  "SYSTEMID",
  "AUTOROTATE"
};

#define RoverModeTextCount 17
const char * RoverModeText[RoverModeTextCount] =
{
  "MANUAL",
  "ACRO",
  "XXX",
  "STEERING",
  "HOLD",
  "LOITER",
  "FOLLOW",
  "SIMPLE",
  "XXX",
  "XXX",
  "AUTO",
  "RTL",
  "SMART_RTL",
  "XXX",
  "XXX",
  "GUIDED",
  "INIT"
};

#endif // #ifndef _TEXTS_H_
