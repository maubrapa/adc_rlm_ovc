//
//  main.cpp
//  roadLaneMarking
//
//  Created by Mauricio de Paula on 28/07/14.
//  Copyright (c) 2014 ___HOME___. All rights reserved.
//

#include <iostream>
#include "ConfigParameters.h"
#include "FileStorageComplements.h"
#include "RoadMark.h"

int main(int argc, const char * argv[]) {

    /* Read the XML config file */
    if ( !( argc == 3) ) {
        std::cerr << ">>> Syntax <<<" << endl;
        std::cerr << "./adc_rlm_ovc <XML_config_file> <clipName>" << endl;
        return 1;
    }

    ConfigParameters config;
    std::string configFile = argv[1];
    std::string videoAttribute = argv[2];
    FileStorage fs;
    fs.open(configFile, FileStorage::READ);
    fs[videoAttribute] >> config;

    /* Deal with empty config or wrong path and/or file name */
    if(config.h == 0 ) {
    std::cerr << "Config file empty or wrong file name!" << endl;
        return 2;
    }
    
    /* Instance*/
    RoadMark* obj = new RoadMark(config);
    obj->classifyVideo();
    
    return 0;
}
