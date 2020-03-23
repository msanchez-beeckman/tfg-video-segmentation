
#ifndef TFG_VIDEO_SEGMENTATION_CMDPARSER_H
#define TFG_VIDEO_SEGMENTATION_CMDPARSER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <cassert>
#include <getopt.h>

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

#include <sys/types.h>
#include <unistd.h>

//
//! Parser
//

//! structure for parameters and options which are
//! optional in the code or they already have a default value
typedef struct optstruct
{
    char *gp;           //! string of two letters  "a:"  as necessary for using getopt afterwards
                        //! the ":" indicates that the activation of the option requires a value for the parameter
                        //! and "a" that this option is activated by "-a " in the command
    
    int flag;           //! flag indicating that the option has been activated
    
    char *defvalue;     //! default value for this parameter if not modified by console
    
    char *value;        //! value of the associated parameter to current option
    
    char *comment;      //! comment that appears by console
    	
} OptStruct;



//! structure for necessary parameters of the method
typedef struct parstruct
{
    char *name;
    char *value;       //! value of the parameter
    char *comment;     //! comment that appears by console
    
} ParStruct;



int parsecmdline(char *pname,
                 char *function,
                 int argc, char **argv,
                 std::vector<OptStruct*> & opt,
                 std::vector<ParStruct*> & par);

#endif //TFG_VIDEO_SEGMENTATION_CMDPARSER_H