disp('Compiling S-function from JSBSim...');
mex -v -O -R2017b COMPFLAGS='$COMPFLAGS /DJSBSIM_STATIC_LINK'  JSBSim_SFunction.cpp JSBSimInterface.cpp -I"..\src" -L"..\build\src\Release" -lJSBSim wsock32.lib ws2_32.lib
disp('Finished.')
