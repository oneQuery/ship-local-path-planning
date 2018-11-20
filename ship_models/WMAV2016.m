function WMAV2016_model = WMAV2016()
%WMAV2016 
C_nu = [-1.0191 0 0 ;
     0 0.0161 -0.0052 ;
     0 8.2861 -0.9860] ;
 
C_tau = [0.0028 0 ;
     0 0.0002 ;
     0 0.0307] ;
 
WMAV2016_model.C_nu = C_nu ;
WMAV2016_model.C_tau = C_tau ;
end

