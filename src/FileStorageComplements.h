//
//  FileStorageComplements.h
//  lane
//
//  Created by Maurício de Paula on 11/29/12.
//  Copyright (c) 2012 UFRGS. All rights reserved.
//

#ifndef lane_FileStorageComplements_h
#define lane_FileStorageComplements_h

//These write and read functions must be defined for the serialization in FileStorage to work
void write(FileStorage& fs, const std::string&, const ConfigParameters& x) {
    x.writeConfigParameters(fs);
}

void read(const FileNode& node, ConfigParameters& x, const ConfigParameters& default_value = ConfigParameters()) {
    if(node.empty())
        x = default_value;
    else
        x.readConfigParameters(node);
}

// This function will print our custom class to the console
ostream& operator<<(ostream& out, const ConfigParameters& m) {
    out << "{" << endl;
    
    out << ">>> General configuration <<<" << endl;
	out << "path_in = " << m.path_in << ", " << endl;
    out << "path_out = " << m.path_out << ", " << endl;
    out << "fileName = " << m.fileName << ", " << endl;
    out << "type_lane_marking = " << m.type_lane_marking << ", " << endl;
	out << "framesToTrain = " << m.framesToTrain << ", " << endl;
    out << "framesToTest = " << m.framesToTest << ", " << endl;
	out << "initFrame = " << m.initFrame << ", " << endl;
	out << "refLane = " << m.refLane << ", " << endl;
	out << "roi = [" << m.u << ", " << m.v << ", " << m.u+m.w << ", " << m.v+m.h << "], " << endl;
	out << "vSweep = [" << m.iV << ", " << m.eV << "], " << endl;
    out << "speedVeh = " << m.speedVeh << endl;

    out << ">>> Intrinsic parameters <<<" << endl;
	out << "f = " << m.f << ", " << endl;
	out << "c = " << m.c << ", " << endl;
    
    out << ">>> Extrinsic parameters <<<" << endl;
    out << "alpha = " << m.extrinsic.alpha << endl;
    out << "beta = " << m.extrinsic.beta << endl;
    out << "h = " << m.extrinsic.h << endl;
	out << "}";
    return out;
    }

#endif
