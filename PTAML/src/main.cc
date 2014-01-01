// This is the main entry point for PTAM
//
// This file parses command line arguments and calls System.run()

#include "System.h"
#include "VideoSource.h"
#include <gvars3/instances.h>
#include <stdlib.h>
#include <iostream>
#include <memory>

using namespace std;
using namespace GVars3;
using namespace PTAMM;

string settingsFilename = "settings.cfg";
int videoSourceSizeWidth = 640;
int videoSourceSizeHeight = 480;

void ParseOptions(int argc, char *argv[])
{
  for (int i = 1; i < argc; i++) {
    if (i < argc - 2 && !strcmp(argv[i], "-size")){
      videoSourceSizeWidth = atoi(argv[i+1]);
      videoSourceSizeHeight = atoi(argv[i+2]);
      i += 2;
      cout << "WindowSize : " << videoSourceSizeWidth << "x" << videoSourceSizeHeight << endl;
    } else if (i < argc - 1 && !strcmp(argv[i], "-config")){
      settingsFilename = argv[i+1];
      cout << "Config file : " << settingsFilename << endl;
      i++;
    }
  }
}

int main(int argc, char *argv[])
{
  cout << "  Welcome to PTAM " << endl;
  cout << "  --------------- " << endl;
  cout << endl;

  GUI.parseArguments(argc, argv);
  GUI.LoadFile(settingsFilename);
  GUI.StartParserThread(); // Start parsing of the console input
  atexit(GUI.StopParserThread);

  ParseOptions(argc, argv);

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
