#include "Alvar_bind.h"
//#include "Contours.h"
#include "Matrix.h"



extern "C" void
init(Handle<Object> target) {
    HandleScope scope;
    Alvar::Init(target);
   // Contour::Init(target);
    Matrix::Init(target);

};

NODE_MODULE(alvar, init)
