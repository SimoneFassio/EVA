#include "PolePlacementControl.h"
#include <iostream>


// Constructor
ControlSystem::ControlSystem(double minForce, double maxForce, double minError, double weight, double buoyancy):
    
    minForce(minForce),
    maxForce(maxForce),  
    minError(minError),
    weight(weight),
    buoyancy(buoyancy)  
   
{

    this->old2DOFReference= 0.0;
    this->oldReference= 0.0;
    this->oldAntiWindUpSignal= 0.0;
    this->old2AntiWindUpSignal= 0.0;
    this->oldForce= 0.0;
    this->old2Force= 0.0;       

}


// Computes the output of the Z controller
double ControlSystem::calculateZ(double reference, double measurement)
{
    double error = reference - measurement;
    //Check whether the error is too small
    if(error<(this->minError) && error> -(this->minError))
    {
        return 0.0;
    }
    // 2DOF DIOPHONTAINE CONTROLLER WITH ANTI WIND-UP
    // From now on all the coefficients come from the antiZ-transform of controller transfer functions
    
    //2DOF control
    double DOF2Reference = 0.970734016537625 * old2DOFReference + 0.103591752972112*reference - 0.074325769509759*oldReference;

    //Error signal
    double DOF2error = DOF2Reference - measurement;
    
    //1DOF controller with anti wind-up scheme
    double antiWindUpSignal = 1.948785785245086*oldAntiWindUpSignal - 0.949428121819122*old2AntiWindUpSignal + 0.271858391165384e-4*oldForce - 0.272372308555764e-4*old2Force;
    double infSignal = DOF2error - antiWindUpSignal;
    double unSatForce = infSignal * 1.249882930717357e4; // 1.249882930717357e4 = cz_inf

    if(unSatForce > maxForce)
    {
        unSatForce = maxForce;
    }
    else if(unSatForce < minForce)
    {
        unSatForce = minForce;
    }

    double Force = unSatForce - weight + buoyancy; 
    
    // Controller memory
    old2DOFReference = DOF2Reference;
    oldReference =  reference;
    old2AntiWindUpSignal =  oldAntiWindUpSignal;
    oldAntiWindUpSignal = antiWindUpSignal;
    old2Force = oldForce;
    oldForce = unSatForce; 
    

    return Force;
}

// Computes the output of the PITCH controller
double ControlSystem::calculatePitch(double reference, double measurement)
{
    double error = reference - measurement;
    //Check whether the error is too small
    if(error<(this->minError) && error> -(this->minError))
    {
        return 0.0;
    }
    // 2DOF DIOPHONTAINE CONTROLLER WITH ANTI WIND-UP
    // From now on all the coefficients come from the antiZ-transform of controller transfer functions
    
    //2DOF control
    double DOF2Reference = 0.970734016537625 * old2DOFReference + 0.103591752972112*reference - 0.074325769509759*oldReference;

    //Error signal
    double DOF2error = DOF2Reference - measurement;
    
    //1DOF controller with anti wind-up scheme
    double antiWindUpSignal = 1.948785785245086*oldAntiWindUpSignal - 0.949428121819122*old2AntiWindUpSignal + 0.271858391165384e-4*oldForce - 0.272372308555764e-4*old2Force;
    double infSignal = DOF2error - antiWindUpSignal;
    double unSatForce = infSignal * 1.249882930717357e4; // 1.249882930717357e4 = cz_inf

    if(unSatForce > maxForce)
    {
        unSatForce = maxForce;
    }
    else if(unSatForce < minForce)
    {
        unSatForce = minForce;
    }

    double Force = unSatForce - weight + buoyancy; 
    
    // Controller memory
    old2DOFReference = DOF2Reference;
    oldReference =  reference;
    old2AntiWindUpSignal =  oldAntiWindUpSignal;
    oldAntiWindUpSignal = antiWindUpSignal;
    old2Force = oldForce;
    oldForce = unSatForce; 
    

    return Force;
}

// Computes the output of the ROLL controller
double ControlSystem::calculateRoll(double reference, double measurement)
{
    double error = reference - measurement;
    
    //1DOF controller with anti wind-up scheme
    double antiWindUpSignal = 1.948785785245086*oldAntiWindUpSignal - 0.949428121819122*old2AntiWindUpSignal + 0.271858391165384e-4*oldForce - 0.272372308555764e-4*old2Force;
    double infSignal = error - antiWindUpSignal;
    double unSatForce = infSignal * 1.249882930717357e4; // 1.249882930717357e4 = cz_inf

    if(unSatForce > maxForce)
    {
        unSatForce = maxForce;
    }
    else if(unSatForce < minForce)
    {
        unSatForce = minForce;
    }

    double Force = unSatForce - weight + buoyancy; 
    
    // Controller memory
    old2AntiWindUpSignal =  oldAntiWindUpSignal;
    oldAntiWindUpSignal = antiWindUpSignal;
    old2Force = oldForce;
    oldForce = unSatForce; 
    

    return Force;
}