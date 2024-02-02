/*
Copyright (c) 2009, Brian Mills
All rights reserved.

Copyright (c) 2021, Agostino De Marco, Elia Tarasov, Michal Podhradsky, Tilda Sikström

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "JSBSimInterface.h"
#include <models/FGAircraft.h>
#include <models/FGAccelerations.h>
#include <math/FGQuaternion.h>

/* 2021-07-08 compiles with JSBSim 1.1.6
 */

JSBSimInterface::JSBSimInterface(int numOutputPorts)
{
	_ac_model_loaded = false;
	fdmExec = new FGFDMExec();
	pm = fdmExec->GetPropertyManager().get();
	propagate = fdmExec->GetPropagate().get();
	accel = fdmExec->GetAccelerations().get();
	auxiliary = fdmExec->GetAuxiliary().get();
	aerodynamics = fdmExec->GetAerodynamics().get();
	propulsion = fdmExec->GetPropulsion().get();
	fcs = fdmExec->GetFCS().get();
	ic = new FGInitialCondition(fdmExec);
	for (int i = 0; i < numOutputPorts; i++) {
		std::vector<FGPropertyNode*> emptyVector;
		outputPorts.push_back(emptyVector);
	}
	//verbosityLevel = JSBSimInterface::eSilent;
	LoadSettings();
}

JSBSimInterface::JSBSimInterface(double dt, int numOutputPorts)
{
	_ac_model_loaded = false;
	fdmExec = new FGFDMExec();
	fdmExec->Setdt(dt);
	mexPrintf("Simulation dt set to %f\n",fdmExec->GetDeltaT());
	pm = fdmExec->GetPropertyManager().get();
	propagate = fdmExec->GetPropagate().get();
	accel = fdmExec->GetAccelerations().get();
	auxiliary = fdmExec->GetAuxiliary().get();
	aerodynamics = fdmExec->GetAerodynamics().get();
	propulsion = fdmExec->GetPropulsion().get();
	fcs = fdmExec->GetFCS().get();
	ic = new FGInitialCondition(fdmExec);
	for (int i = 0; i < numOutputPorts; i++) {
		std::vector<FGPropertyNode*> emptyVector;
		outputPorts.push_back(emptyVector);
	}
	//verbosityLevel = JSBSimInterface::eSilent;
	LoadSettings();	
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
JSBSimInterface::~JSBSimInterface(void)
{
    delete fdmExec;
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::LoadSettings()
{	
    std::string current_config_path = "./config.ini"; 
    std::map<std::string, std::string> iniData = parseIniFile(current_config_path);
    
	if(!iniData.empty())
	{
		for (const auto& pair : iniData) {
			std::cout << pair.first << " == " << pair.second << std::endl;
			 if(pair.first == "Paths.aircraft")
			 {
			 	fdmExec->SetAircraftPath(SGPath(pair.second));
				std::cout << "AIRCRAFT  " << fdmExec->GetAircraftPath() << std::endl;
				
			 }
			 else if (pair.first == "Paths.engine")
			 {
			 	fdmExec->SetEnginePath(SGPath(pair.second));
				std::cout << "ENGINE  " << fdmExec->GetEnginePath() << std::endl;
			 }
			else if (pair.first == "Paths.systems")
			 {
			 	fdmExec->SetSystemsPath(SGPath(pair.second));
				std::cout << "SYSTEMS  " << fdmExec->GetSystemsPath() << std::endl;
			 }
		}
		std::cout << "------------------------------------"  << std::endl;
		return true;
	}
	else
	{
		    if (!fdmExec->SetAircraftPath (SGPath("aircraft"))) return false;  
    		if (!fdmExec->SetEnginePath   (SGPath("engine"))) return false;
    		if (!fdmExec->SetSystemsPath  (SGPath("systems"))) return false;
			return true;
	}
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::OpenAircraft(const string& acName)
{

	if (!fdmExec->GetAircraft()->GetAircraftName().empty()) return false;

    if ( ! fdmExec->LoadModel(acName)) return false;

	_ac_model_loaded = true;

  	return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::OpenScript(const SGPath& script, double delta_t, const SGPath& initfile)
{
    if (!fdmExec->LoadScript(script, delta_t, initfile)) return false;

    if (!fdmExec->RunIC()) return false;

    return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::LoadIC(SGPath ResetName)
{

    auto IC = fdmExec->GetIC(); 
	
    if (!IC->Load(ResetName)) return false;

    if (!fdmExec->RunIC()) return false;
	
	return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void JSBSimInterface::Update()
{
    fdmExec->Run();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::AddInputPropertyNode(std::string property)
{

	FGPropertyNode* node = pm->GetNode(property);
	if (node == NULL || !node->getAttribute(FGPropertyNode::Attribute::WRITE)) return false;

	inputPort.push_back(node);
	return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::AddWeatherPropertyNode(std::string property)
{
	
	if (!(property.substr(0, std::string("atmosphere/").size()) == std::string("atmosphere/"))) return false;

	FGPropertyNode* node = pm->GetNode(property);
	if (node == NULL || !node->getAttribute(FGPropertyNode::Attribute::WRITE)) return false;

	weatherPort.push_back(node);
	return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::AddOutputPropertyNode(std::string property, const int outputPort)
{
	
	if (outputPort >= outputPorts.size()) return false;

	FGPropertyNode* node = pm->GetNode(property);
	if (node == NULL || !node->getAttribute(FGPropertyNode::Attribute::READ)) return false;

	outputPorts.at(outputPort).push_back(node);
	return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::CopyInputControlsToJSBSim(std::vector<double> controls) {
    // TODO: error handling if controls is not correct size. 
    
	if (!fdmExec) return false;

	FGPropertyNode* node;
	for (int i = 0; i < inputPort.size(); i++) {
		node = inputPort.at(i);
		switch (node->getType()) {
			case simgear::props::BOOL:
				node->setBoolValue(controls[i]);
				break;
			case simgear::props::INT:
				node->setIntValue(controls[i]);
				break;
			case simgear::props::LONG:
				node->setLongValue(controls[i]);
				break;
			case simgear::props::FLOAT:
				node->setFloatValue(controls[i]);
				break;
			case simgear::props::DOUBLE:
				node->setDoubleValue(controls[i]);
				break;
			default:
				return false;
		}
	}

    return true; 
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::CopyInputWeatherToJSBSim(std::vector<double> weather) {
    // TODO: error handling if weather is not correct size. 
    
	if (!fdmExec) return false;

	FGPropertyNode* node;
	for (int i = 0; i < weatherPort.size(); i++) {
		node = weatherPort.at(i);
		switch (node->getType()) {
			case simgear::props::BOOL:
				node->setBoolValue(weather[i]);
				break;
			case simgear::props::INT:
				node->setIntValue(weather[i]);
				break;
			case simgear::props::LONG:
				node->setLongValue(weather[i]);
				break;
			case simgear::props::FLOAT:
				node->setFloatValue(weather[i]);
				break;
			case simgear::props::DOUBLE:
				node->setDoubleValue(weather[i]);
				break;
			default:
				return false;
		}
	}

    return true; 
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bool JSBSimInterface::CopyOutputsFromJSBSim(double *stateArray, const int outputPort) {
	
	if (outputPort >= outputPorts.size()) {
		mexPrintf("Output port selected is out of bounds.\n");
	}
	
	FGPropertyNode* node;
	std::vector<FGPropertyNode*> port = outputPorts.at(outputPort);
	for (int i = 0; i < port.size(); i++) {
		node = port.at(i);
		switch (node->getType()) {
			case simgear::props::BOOL:
				stateArray[i] = node->getBoolValue();
				break;
			case simgear::props::INT:
				stateArray[i] = node->getIntValue();
				break;
			case simgear::props::LONG:
				stateArray[i] = node->getLongValue();
				break;
			case simgear::props::FLOAT:
				stateArray[i] = node->getFloatValue();
				break;
			case simgear::props::DOUBLE:
				stateArray[i] = node->getDoubleValue();
				break;
			default:
				return false;
		}
	}
	
	return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
std::map<std::string, std::string> JSBSimInterface::parseIniFile(const std::string& filename) {
    std::map<std::string, std::string> iniData;
    std::ifstream inFile(filename);

    if (!inFile) {
        std::cerr << "Error opening file: " << filename << ". Default directories for settings will be loaded" << std::endl;
        return iniData; // Return an empty map if file opening fails
    }

    std::string line;
    std::string currentSection;

    while (std::getline(inFile, line)) {
        // Remove leading and trailing whitespaces
        line.erase(line.find_last_not_of(" \t") + 1);
        line.erase(0, line.find_first_not_of(" \t"));

        if (line.empty() || line[0] == ';' || line[0] == '#') {
            // Ignore empty lines or comments
            continue;
        } else if (line[0] == '[' && line.back() == ']') {
            // Handle section headers
            currentSection = line.substr(1, line.size() - 2);
        } else {
            // Parse key-value pairs
            std::size_t equalPos = line.find('=');
            if (equalPos != std::string::npos) {
                std::string key = line.substr(0, equalPos);
                std::string value = line.substr(equalPos + 1);
                iniData[currentSection + "." + key] = value;
            }
        }
    }

    inFile.close();
    return iniData;
}



void JSBSimInterface::exportLTI()
{
	FGLinearization lin(fdmExec);

	mxArray* mxMatrixA = exportMatrixToMatlab(lin.GetSystemMatrix());
    int statusA = mexPutVariable("base", "A", mxMatrixA);
	mxDestroyArray(mxMatrixA);

	mxArray* mxMatrixB = exportMatrixToMatlab(lin.GetInputMatrix());
    int statusB = mexPutVariable("base", "B", mxMatrixB);
	mxDestroyArray(mxMatrixB);

	mxArray* mxMatrixC = exportMatrixToMatlab(lin.GetOutputMatrix());
    int statusC = mexPutVariable("base", "C", mxMatrixC);
	mxDestroyArray(mxMatrixC);

	mxArray* mxMatrixD = exportMatrixToMatlab(lin.GetFeedforwardMatrix());
    int statusD = mexPutVariable("base", "D", mxMatrixD);
	mxDestroyArray(mxMatrixD);


	mxArray* mxStateNames = exportStringVectorToMatlab(lin.GetStateNames());
	int statusSN = mexPutVariable("base", "state_names", mxStateNames);
	mxDestroyArray(mxStateNames);

	mxArray* mxInputNames = exportStringVectorToMatlab(lin.GetInputNames());
	int statuIN = mexPutVariable("base", "input_names", mxInputNames);
	mxDestroyArray(mxInputNames);

	mxArray* mxOutputNames = exportStringVectorToMatlab(lin.GetOutputNames());
	int statusON = mexPutVariable("base", "output_names", mxOutputNames);
	mxDestroyArray(mxOutputNames);
}

// Function to export a vector of strings to MATLAB
mxArray* JSBSimInterface::exportStringVectorToMatlab(const std::vector<std::string>& stringVector) {
    mwSize numStrings = stringVector.size();

    // Create cell array mxArray
    mxArray* mxCellArray = mxCreateCellMatrix(1, numStrings);
    if (mxCellArray == nullptr) {
        mexErrMsgTxt("Memory allocation failed for cell array.");
        return nullptr;
    }

    // Fill cell array with strings
    for (mwIndex i = 0; i < numStrings; ++i) {
        mxArray* mxString = mxCreateString(stringVector[i].c_str());
        if (mxString == nullptr) {
            mxDestroyArray(mxCellArray);
            mexErrMsgTxt("Memory allocation failed for string.");
            return nullptr;
        }
        mxSetCell(mxCellArray, i, mxString);
    }

    return mxCellArray;
}

// Function to export a matrix to MATLAB
mxArray* JSBSimInterface::exportMatrixToMatlab(const std::vector<std::vector<double>>& matrix) {
    mwSize numRows = matrix.size();
    mwSize numCols = (numRows > 0) ? matrix[0].size() : 0;

    // Create mxArray
    mxArray* mxMatrix = mxCreateDoubleMatrix(numRows, numCols, mxREAL);
    if (mxMatrix == nullptr) {
        // Handle memory allocation failure
        mexErrMsgTxt("Memory allocation failed.");
        return nullptr;
    }

    // Copy data to mxArray
    double* dataPtr = mxGetPr(mxMatrix);
    for (mwSize i = 0; i < numRows; ++i) {
        for (mwSize j = 0; j < numCols; ++j) {
            dataPtr[i + j * numRows] = matrix[i][j];
        }
    }

    return mxMatrix;
}