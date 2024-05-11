#ifndef _POLEPLACEMENTCONTROL_H
#define _POLEPLACEMENTCONTROL_H

class ControlSystem
{
    public:

        //Constructor
        ControlSystem(double minForce, double maxForce, double minError, double weight, double buoyancy);

        
        //Returns the output of the controller given a reference signal and the actual measured state of the system
        double calculateZ(double reference, double measurement);
        double calculatePitch(double reference, double measurement);
        double calculateRoll(double reference, double measurement);
    private:

        const double minForce;  //for control input saturation purposes
        const double maxForce;  //for control input saturation purposes
        const double minError;  //It prevents the controller from activating due to small measurement errors
        const double weight;
        const double buoyancy;
        
        //Controller memory
        double old2DOFReference;
        double oldReference;	
        double oldAntiWindUpSignal;
        double old2AntiWindUpSignal;
        double oldForce;
        double old2Force;


};

#endif