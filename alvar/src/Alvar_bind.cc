#include "Alvar_bind.h"
#include "Matrix.h"
#include <node.h>


v8::Persistent<FunctionTemplate> Alvar::constructor;

void Alvar::Init(Handle<Object> target) {
  HandleScope scope;
   char out [21];
  int n = sprintf(out, "%i.%i", CV_MAJOR_VERSION, CV_MINOR_VERSION);
  target->Set(String::NewSymbol("version"), String::New(out, n)); 


  //Class
  v8::Local<v8::FunctionTemplate> m = v8::FunctionTemplate::New(New);
  m->SetClassName(v8::String::NewSymbol("Alvar"));

  // Constructor
  constructor = Persistent<FunctionTemplate>::New(m);
  constructor->InstanceTemplate()->SetInternalFieldCount(1);
  constructor->SetClassName(String::NewSymbol("Alvar"));

  // Prototype
  //Local<ObjectTemplate> proto = constructor->PrototypeTemplate();


  NODE_SET_PROTOTYPE_METHOD(constructor, "readImage", ReadImage);
  NODE_SET_PROTOTYPE_METHOD(constructor, "readHomography", ReadHomography);
  NODE_SET_PROTOTYPE_METHOD(constructor, "treatImageBuffer", treatImageBuffer);
  NODE_SET_PROTOTYPE_METHOD(constructor, "GetNumMarkerDetected", GetNumMarkerDetected);
  NODE_SET_PROTOTYPE_METHOD(constructor, "GetMarkersCoord", GetMarkersCoord);
  NODE_SET_PROTOTYPE_METHOD(constructor, "GetHomography", GetHomography);
  NODE_SET_PROTOTYPE_METHOD(constructor, "GetDepthPix", GetDepthPix);
  


  target->Set(String::NewSymbol("Alvar"), m->GetFunction());
  cout<<"Init Done"<<endl;
}  

Handle<Value> Alvar::New(const Arguments &args) {
  HandleScope scope;

  Alvar* obj = new Alvar();

  obj->Wrap(args.Holder());
  return scope.Close(args.Holder());  
  // obj->Wrap(args.This());
  // return args.This();

}

Alvar::Alvar() {
   
marker_size=0.135;
    cout<<"Lecture Calib"<<endl;

        if (cam.SetCalib("camera_ardrone_front.xml", 640, 360)) {
            cout<<cam.calib_K_data[0][0]<<endl;
            cout<<" [Ok]"<<endl;
        } else {
            cam.SetRes(640, 480);
            cout<<" [Fail] Could not load calibration file "<<endl;
        }


}  


Handle<Value>
Alvar::treatImageBuffer(const Arguments &args) {
  SETUP_FUNCTION(Alvar)


  try{
    
    Local<Object> im_h = Matrix::constructor->GetFunction()->NewInstance();
    Matrix *img = ObjectWrap::Unwrap<Matrix>(im_h);
    cv::Mat mat;

    uint8_t *buf = (uint8_t *) Buffer::Data(args[0]->ToObject());
    unsigned len = Buffer::Length(args[0]->ToObject());
    
    cv::Mat *mbuf = new cv::Mat(len, 1, CV_64FC1, buf);
    mat = cv::imdecode(*mbuf, -1);
          
    if (mat.empty()){
      return v8::ThrowException(v8::Exception::TypeError(v8::String::New("Error loading file")));
    }
    
    IplImage image = mat;
    

    bool flip_image = (image.origin?true:false);
    if (flip_image) {
        cvFlip(&image);
        image.origin = !image.origin;
    }


     Local<Array> pos=Array::New(12);
     pos->Set(Integer::New(0),Number::New(-1));
        pos->Set(Integer::New(1),Number::New(0));
        pos->Set(Integer::New(2),Number::New(0));
        pos->Set(Integer::New(3),Number::New(0));
        int id_pos = 0;
    self->marker_detector.SetMarkerSize(self->marker_size); // for marker ids larger than 255, set the content resolution accordingly
    //static MarkerDetector<MarkerArtoolkit> marker_detector;
    //marker_detector.SetMarkerSize(2.8, 3, 1.5);

    // Here we try to use RGBA just to make sure that also it works...
    //cvCvtColor(image, rgba, CV_RGB2RGBA);
    self->marker_detector.Detect(&image, &(self->cam), true, true);
    
    for (size_t i=0; i<self->marker_detector.markers->size(); i++) {
        if (i >= 32) break;
        
        Pose p = (*(self->marker_detector.markers))[i].pose;
        
        
        int id = (*(self->marker_detector.markers))[i].GetId();

        // Update the size of the returned array
        if(i>0)
        {
          pos->Set( String::NewSymbol("length"), v8::Number::New(pos->Length()+12));
        }
      
        pos->Set(Integer::New(id_pos++),Number::New(id));
        pos->Set(Integer::New(id_pos++),Number::New(p.translation[0]));
        pos->Set(Integer::New(id_pos++),Number::New(p.translation[1]));
        pos->Set(Integer::New(id_pos++),Number::New(p.translation[2]));
        
        for(int j=0;j<4;j++)
        {
          pos->Set(Integer::New(id_pos++),Number::New((*(self->marker_detector.markers))[i].marker_corners_img[j].x));
          pos->Set(Integer::New(id_pos++),Number::New((*(self->marker_detector.markers))[i].marker_corners_img[j].y));
        }
      
       
    }

    if (flip_image) {
        cvFlip(&image);
        image.origin = !image.origin;
    }




    img->mat = mat;


    return scope.Close(pos);

  } catch( cv::Exception& e ){
      const char* err_msg = e.what();
      return v8::ThrowException(v8::Exception::Error(v8::String::New(err_msg)));
  }
}; 






Handle<Value>
Alvar::GetNumMarkerDetected(const Arguments &args) {
    SETUP_FUNCTION(Alvar)

    try
    {
       Local<Number> nmarkers = Number::New(self->marker_detector.markers->size());
      return scope.Close(nmarkers);
    }
    catch( cv::Exception& e ){
      const char* err_msg = e.what();
      return v8::ThrowException(v8::Exception::Error(v8::String::New(err_msg)));
    }
}


Handle<Value>
Alvar::GetMarkersCoord(const Arguments &args) {
    SETUP_FUNCTION(Alvar)

    try
    {
          Local<Array> pos=Array::New(12);

          pos->Set(Integer::New(0),Number::New(-1));
          pos->Set(Integer::New(1),Number::New(0));
          pos->Set(Integer::New(2),Number::New(0));
          pos->Set(Integer::New(3),Number::New(0));
          int id_pos = 0;

          for (size_t i=0; i<self->marker_detector.markers->size(); i++) {
            if (i >= 32) break;

            Pose p = (*(self->marker_detector.markers))[i].pose;


            int id = (*(self->marker_detector.markers))[i].GetId();

              // Update the size of the returned array
            if(i>0)
            {
              pos->Set( String::NewSymbol("length"), v8::Number::New(pos->Length()+12));
            }
            
            pos->Set(Integer::New(id_pos++),Number::New(id));
            pos->Set(Integer::New(id_pos++),Number::New(p.translation[0]));
            pos->Set(Integer::New(id_pos++),Number::New(p.translation[1]));
            pos->Set(Integer::New(id_pos++),Number::New(p.translation[2]));

            for(int j=0;j<4;j++)
            {
              pos->Set(Integer::New(id_pos++),Number::New((*(self->marker_detector.markers))[i].marker_corners_img[j].x));
              pos->Set(Integer::New(id_pos++),Number::New((*(self->marker_detector.markers))[i].marker_corners_img[j].y));
            }
            

          }



          return scope.Close(pos);
      }
      catch( cv::Exception& e ){
        const char* err_msg = e.what();
        return v8::ThrowException(v8::Exception::Error(v8::String::New(err_msg)));
      }
  }



Handle<Value>
  Alvar::GetDepthPix(const Arguments &args) {
      SETUP_FUNCTION(Alvar)

      //This function expects one argument which is the id of the marker
      // from which the homography is computed

      int target_id = args[0]->NumberValue();
      double hh = -1;
      try
      {
            


                for (size_t i=0; i<self->marker_detector.markers->size(); i++) {
                    if (i >= 32) break;
                    
                    Pose p = (*(self->marker_detector.markers))[i].pose;
                    
                    
                    int id = (*(self->marker_detector.markers))[i].GetId();
                   
                    cv::Mat H= cv::Mat::zeros(3, 3, CV_32F);
                    cv::Mat w, u, vt;
                   

                    if(id == target_id)
                    {
                       vector<cv::Point2f> pts_src;
                        vector<cv::Point2f> pts_dst;
                        for(int j=0;j<4;j++)
                        {
                         pts_dst.push_back(cv::Point2f((*(self->marker_detector.markers))[i].marker_corners_img[j].x,(*(self->marker_detector.markers))[i].marker_corners_img[j].y)); 
                        }
                        pts_src.push_back(cv::Point2f(270.0,130.0));
                        pts_src.push_back(cv::Point2f(270.0,230.0));
                        pts_src.push_back(cv::Point2f(370.0,230.0));
                        pts_src.push_back(cv::Point2f(370.0,130.0));

                          H = cv::findHomography(pts_src, pts_dst, 0);
                          cv::SVD::compute(H, w, u, vt);

                          hh = w.at<double>(1);
                    }

       
            }

          Local<Number> depthpix = Number::New(hh);
          return scope.Close(depthpix);
    }
    catch( cv::Exception& e ){
      const char* err_msg = e.what();
      return v8::ThrowException(v8::Exception::Error(v8::String::New(err_msg)));
    }
}

  Handle<Value>
  Alvar::GetHomography(const Arguments &args) {
      SETUP_FUNCTION(Alvar)

      //This function expects one argument which is the id of the marker
      // from which the homography is computed

      int target_id = args[0]->NumberValue();

      try
      {
            Local<Array> homography=Array::New(9+8);


                for (size_t i=0; i<self->marker_detector.markers->size(); i++) {
                    if (i >= 32) break;
                    
                    Pose p = (*(self->marker_detector.markers))[i].pose;
                    
                    
                    int id = (*(self->marker_detector.markers))[i].GetId();
                   
                    cv::Mat H= cv::Mat::zeros(3, 3, CV_32F);
                    if(id == target_id)
                    {
                       vector<cv::Point2f> pts_src;
                        vector<cv::Point2f> pts_dst;
                        for(int j=0;j<4;j++)
                        {
                         pts_dst.push_back(cv::Point2f((*(self->marker_detector.markers))[i].marker_corners_img[j].x,(*(self->marker_detector.markers))[i].marker_corners_img[j].y)); 
                        }
                        pts_src.push_back(cv::Point2f(270.0,130.0));
                        pts_src.push_back(cv::Point2f(270.0,230.0));
                        pts_src.push_back(cv::Point2f(370.0,230.0));
                        pts_src.push_back(cv::Point2f(370.0,130.0));

                          H = cv::findHomography(pts_src, pts_dst, 0);
                         int id_h=0;
                         for(int j=0;j<3;j++)
                          for(int k=0;k<3;k++)
                         homography->Set(Integer::New(id_h++),Number::New(H.at<float>(k,j)));

                          for(int j=0;j<4;j++)
                          {
                            homography->Set(Integer::New(id_h++),Number::New(pts_dst[j].x));
                            homography->Set(Integer::New(id_h++),Number::New(pts_dst[j].y));
                          }

              }

       
    }


          return scope.Close(homography);
    }
    catch( cv::Exception& e ){
      const char* err_msg = e.what();
      return v8::ThrowException(v8::Exception::Error(v8::String::New(err_msg)));
    }
}

Handle<Value>
Alvar::ReadImage(const Arguments &args) {
  SETUP_FUNCTION(Alvar)


  try{
    
    Local<Object> im_h = Matrix::constructor->GetFunction()->NewInstance();
    Matrix *img = ObjectWrap::Unwrap<Matrix>(im_h);
    cv::Mat mat;


    if (args[0]->IsNumber() && args[1]->IsNumber()){
      int width, height;

      width = args[0]->Uint32Value();
      height = args[1]->Uint32Value();    
      mat = *(new cv::Mat(width, height, CV_64FC1));

    } else if (args[0]->IsString()) {
      
      std::string filename = std::string(*v8::String::AsciiValue(args[0]->ToString()));
      mat = cv::imread(filename);

    } else if (Buffer::HasInstance(args[0])){
     	uint8_t *buf = (uint8_t *) Buffer::Data(args[0]->ToObject());
     	unsigned len = Buffer::Length(args[0]->ToObject());
      
  	 	cv::Mat *mbuf = new cv::Mat(len, 1, CV_64FC1, buf);
      mat = cv::imdecode(*mbuf, -1);
            
      if (mat.empty()){
        return v8::ThrowException(v8::Exception::TypeError(v8::String::New("Error loading file")));
      }
    }

    IplImage image = mat;
    

    bool flip_image = (image.origin?true:false);
    if (flip_image) {
        cvFlip(&image);
        image.origin = !image.origin;
    }


     Local<Array> pos=Array::New(12);
     pos->Set(Integer::New(0),Number::New(-1));
        pos->Set(Integer::New(1),Number::New(0));
        pos->Set(Integer::New(2),Number::New(0));
        pos->Set(Integer::New(3),Number::New(0));
        int id_pos = 0;
    self->marker_detector.SetMarkerSize(self->marker_size); // for marker ids larger than 255, set the content resolution accordingly
    //static MarkerDetector<MarkerArtoolkit> marker_detector;
    //marker_detector.SetMarkerSize(2.8, 3, 1.5);

    // Here we try to use RGBA just to make sure that also it works...
    //cvCvtColor(image, rgba, CV_RGB2RGBA);
    self->marker_detector.Detect(&image, &(self->cam), true, true);
    
    for (size_t i=0; i<self->marker_detector.markers->size(); i++) {
        if (i >= 32) break;
        
        Pose p = (*(self->marker_detector.markers))[i].pose;
        
        
        int id = (*(self->marker_detector.markers))[i].GetId();
        // pos->Set(Integer::New(id_pos),Number::New(id));
        // id_pos++;
        // pos->Set(Integer::New(id_pos),Number::New(p.translation[0]));
        // id_pos++;
        // pos->Set(Integer::New(id_pos),Number::New(p.translation[1]));
        // id_pos++;
        // pos->Set(Integer::New(id_pos),Number::New(p.translation[2]));
        // id_pos++;
        
        // for(int j=0;j<4;j++)
        // {
        //   pos->Set(Integer::New(id_pos),Number::New((*(self->marker_detector.markers))[i].marker_corners_img[j].x));
        //   id_pos++;
        //   pos->Set(Integer::New(id_pos),Number::New((*(self->marker_detector.markers))[i].marker_corners_img[j].y));
        //   id_pos++;
        // }
        if(i>0)
        pos->Set( String::NewSymbol("length"), v8::Number::New(pos->Length()+12));
      
        pos->Set(Integer::New(id_pos++),Number::New(id));
        pos->Set(Integer::New(id_pos++),Number::New(p.translation[0]));
        pos->Set(Integer::New(id_pos++),Number::New(p.translation[1]));
        pos->Set(Integer::New(id_pos++),Number::New(p.translation[2]));
        
        for(int j=0;j<4;j++)
        {
          pos->Set(Integer::New(id_pos++),Number::New((*(self->marker_detector.markers))[i].marker_corners_img[j].x));
          pos->Set(Integer::New(id_pos++),Number::New((*(self->marker_detector.markers))[i].marker_corners_img[j].y));
        }
      
      if(id==0)
      {
   //   cout<<"ID:"<<id<<"  pose:"<<p.translation[0]<<","<<p.translation[1]<<","<<p.translation[2]<<","<<p.translation[3]<<","<<endl;
      }
       
    }

    if (flip_image) {
        cvFlip(&image);
        image.origin = !image.origin;
    }




    img->mat = mat;


    return scope.Close(pos);

  } catch( cv::Exception& e ){
      const char* err_msg = e.what();
      return v8::ThrowException(v8::Exception::Error(v8::String::New(err_msg)));
  }
};    


Handle<Value>
Alvar::ReadHomography(const Arguments &args) {
  SETUP_FUNCTION(Alvar)


  try{
    
    Local<Object> im_h = Matrix::constructor->GetFunction()->NewInstance();
    Matrix *img = ObjectWrap::Unwrap<Matrix>(im_h);
    cv::Mat mat;


    if (args[0]->IsNumber() && args[1]->IsNumber()){
      int width, height;

      width = args[0]->Uint32Value();
      height = args[1]->Uint32Value();    
      mat = *(new cv::Mat(width, height, CV_64FC1));

    } else if (args[0]->IsString()) {
      
      std::string filename = std::string(*v8::String::AsciiValue(args[0]->ToString()));
      mat = cv::imread(filename);

    } else if (Buffer::HasInstance(args[0])){
      uint8_t *buf = (uint8_t *) Buffer::Data(args[0]->ToObject());
      unsigned len = Buffer::Length(args[0]->ToObject());
      
      cv::Mat *mbuf = new cv::Mat(len, 1, CV_64FC1, buf);
      mat = cv::imdecode(*mbuf, -1);
            
      if (mat.empty()){
        return v8::ThrowException(v8::Exception::TypeError(v8::String::New("Error loading file")));
      }
    }

    IplImage image = mat;
    

    bool flip_image = (image.origin?true:false);
    if (flip_image) {
        cvFlip(&image);
        image.origin = !image.origin;
    }


     Local<Array> homography=Array::New(9+8);

    self->marker_detector.SetMarkerSize(self->marker_size); // for marker ids larger than 255, set the content resolution accordingly
    //static MarkerDetector<MarkerArtoolkit> marker_detector;
    //marker_detector.SetMarkerSize(2.8, 3, 1.5);

    // Here we try to use RGBA just to make sure that also it works...
    //cvCvtColor(image, rgba, CV_RGB2RGBA);
    self->marker_detector.Detect(&image, &(self->cam), true, false);
    
    for (size_t i=0; i<self->marker_detector.markers->size(); i++) {
        if (i >= 32) break;
        
        Pose p = (*(self->marker_detector.markers))[i].pose;
        
        
        int id = (*(self->marker_detector.markers))[i].GetId();
       
        cv::Mat H= cv::Mat::zeros(3, 3, CV_32F);
        if(id == 0)
        {
           vector<cv::Point2f> pts_src;
            vector<cv::Point2f> pts_dst;
            for(int j=0;j<4;j++)
            {
             pts_dst.push_back(cv::Point2f((*(self->marker_detector.markers))[i].marker_corners_img[j].x,(*(self->marker_detector.markers))[i].marker_corners_img[j].y)); 
            }
            pts_src.push_back(cv::Point2f(270.0,130.0));
            pts_src.push_back(cv::Point2f(270.0,230.0));
            pts_src.push_back(cv::Point2f(370.0,230.0));
            pts_src.push_back(cv::Point2f(370.0,130.0));

              H = cv::findHomography(pts_src, pts_dst, 0);
             int id_h=0;
             for(int j=0;j<3;j++)
              for(int k=0;k<3;k++)
             homography->Set(Integer::New(id_h++),Number::New(H.at<float>(k,j)));

              for(int j=0;j<4;j++)
              {
                homography->Set(Integer::New(id_h++),Number::New(pts_dst[j].x));
                homography->Set(Integer::New(id_h++),Number::New(pts_dst[j].y));
              }

        }

       
    }

    if (flip_image) {
        cvFlip(&image);
        image.origin = !image.origin;
    }




    img->mat = mat;


    return scope.Close(homography);

  } catch( cv::Exception& e ){
      const char* err_msg = e.what();
      return v8::ThrowException(v8::Exception::Error(v8::String::New(err_msg)));
  }
};  
    

