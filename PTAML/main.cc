// Copyright 2009 Isis Innovation Limited
// This is the main extry point for PTAMM
#include <stdlib.h>
#include <iostream>
#include <gvars3/instances.h>
#include "System.h"

using namespace std;
using namespace GVars3;
using namespace PTAMM;

string videoSourceFileName;
string settingsFilename = "settings.cfg";
int videoSourceSizeWidth = 640;
int videoSourceSizeHeight = 480;

// 2011/06/28 takeoka[at]ipc.i.u-tokyo.ac.jp
// Parsing command-line option
void tak_parseoptions(int argc, char *argv[])
{
  for (int i = 1; i < argc; i++) {
    if (string(argv[i]) == "-size"){
      videoSourceSizeWidth = atoi(argv[i+1]);
      videoSourceSizeHeight = atoi(argv[i+2]);
      i += 2;
      cout << "TAK : -size : WindowSize : " << videoSourceSizeWidth << "x" << videoSourceSizeHeight << endl;
    } else if (string(argv[i]) == "-config"){
      settingsFilename = argv[i+1];
      cout << "TAK : -config : Config file : " << settingsFilename << endl;
      i++;
    } else {
      videoSourceFileName = argv[i];
      cout << "TAK : MovieFile-name : " << videoSourceFileName << endl;
    }
  }
}

int main(int argc, char *argv[])
{
  cout << "  Welcome to PTAMM " << endl;
  cout << "  ---------------- " << endl;
  cout << "  Parallel tracking and multiple mapping" << endl;
  cout << "  Copyright (C) Isis Innovation Limited 2009 " << endl;
  cout << endl;

  tak_parseoptions(argc, argv);
  GUI.LoadFile(settingsFilename);
  GUI.StartParserThread(); // Start parsing of the console input
  atexit(GUI.StopParserThread);

  try
  {
    System s;
    s.Run();
  }
  catch(CVD::Exceptions::All& e)
  {
    cout << endl;
    cout << "!! Failed to run system; got exception. " << endl;
    cout << "   Exception was: " << endl;
    cout << e.what << endl;
    return 1;
  }

  return 0;
}



