/*  Proportional–integral–derivative controller (PID controller) is a control loop feedback mechanism.

    PID controller calculates an "error" value as the difference between a measured process variable and a desired setpoint. 
    The controller attempts to minimize the error by adjusting the process control inputs.
    
    Input = Kinematics or sensor input
    Output = PID output
    SetPoint = Desired output
    windupGuard = maximum and minimum constrain on I term
    
    Empty constructor will allow us to declare PID object globally anywhere before the setup() function
    This will allow us to feed PID settings and other variables when they are ready
    (in our case, after EEPROM has been initialized / read into variables)
    
    Pointer arithmetic used to parse terms into PID controller @http://www.cs.umd.edu/class/sum2003/cmsc311/Notes/BitOp/pointer.html
*/

#define deriv_tau 7.9577e-3f

class PID {
    public:
        // Constructor
        PID() {
        };
        
        PID(float* Input, float* Output, float* Setpoint, float* terms) {
            integral = 0.0;
            last_PID_input = 0.0;
            last_error = 0.0;
            last_derivative = 0.0;


            PID_input = Input;
            PID_output = Output;
            PID_setpoint = Setpoint;
            
            Kp = &terms[P];
            Ki = &terms[I];
            Kd = &terms[D];
            
            windupGuard = &terms[WG];        
        };
        
        void Compute() {       
            float derivative;
            unsigned long now;
            float delta_time;
            float error;
            float diff;

            error = *PID_setpoint - *PID_input;

            now = micros();
            delta_time = constrain( (now - last_time) / 1000000.0, 0.002, 0.050 );
            last_time = now;

            if( *Ki )
            {
            	integral = constrain(integral + error * delta_time, -*windupGuard, *windupGuard);
            }
            else
            {
            	integral = 0.0;
            }

    	    if( *Kd )
    	    {
    	    	diff = -(error - last_error); // for compatibility to negative Kd
    	    	// diff = (*PID_input - last_PID_input);
    	    	//derivative = *Kd * diff / delta_time;
    	    	//derivative = *Kd * ( (*PID_input) - (last_PID_input) ) / delta_time;

    	    	// dt1 taulabs
    	    	derivative = last_derivative +  delta_time / ( delta_time + deriv_tau) * (( *Kd * diff / delta_time) - last_derivative);
    	                                                               //   ^ set constant to 1/(2*pi*f_cutoff)
    	    		                                                   //   7.9577e-3  means 20 Hz f_cutoff
    	    }
    	    else
    	    {
    	    	derivative = 0.0;
    	    }
            last_PID_input = *PID_input;
            last_error = error;
            last_derivative = derivative;
            
            *PID_output = (*Kp * error) + (*Ki * integral) + (derivative);
        };
        
        void IntegralReset() {
            integral = 0.0;
        };
    
    private:
        float *PID_input;
        float *PID_output;
        float *PID_setpoint;
        
        float *Kp, *Ki, *Kd;
        float *windupGuard;
        
        float integral;
        float last_PID_input;
        float last_error;
        float last_derivative;
        unsigned long last_time;


}; 
