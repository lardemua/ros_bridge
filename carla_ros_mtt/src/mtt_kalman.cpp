//
// Created by pedro on 14-04-2019.
//


/**************************************************************************************************
 Software License Agreement (BSD License)
 Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
 All rights reserved.
 Redistribution and use in source and binary forms, with or without modification, are permitted
 provided that the following conditions are met:
  *Redistributions of source code must retain the above copyright notice, this list of
   conditions and the following disclaimer.
  *Redistributions in binary form must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other materials provided
   with the distribution.
  *Neither the name of the University of Aveiro nor the names of its contributors may be used to
   endorse or promote products derived from this software without specific prior written permission.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************/
/**
\file
\brief Kalman estimation related functions.
*/

// System Includes
#include <carla_ros_mtt/mtt_kalman.h>

#define CV_MODEL 0

int select_object;
extern double gl;


void AddPointErrorVectors(t_errors*error,double x_inno,double y_inno,double x_resi,double y_resi,double lateral_error)
{
    error->x_innovation[error->position]=x_inno;
    error->y_innovation[error->position]=y_inno;

    error->x_residue[error->position]=x_resi;
    error->y_residue[error->position]=y_resi;

    error->lateral_error[error->position]=lateral_error;

    error->latest=error->position;
    error->position++;

    GetErrorConvariance(error);

    if(error->position==error->max_number_points)
        error->position=0;

    error->next=error->position;

    if(error->number_points<error->max_number_points)
        error->number_points++;
}

void MotionModelsIteration(vector<t_listPtr> &list,t_config& config)
{
    CvMat*x_measurement=cvCreateMat( 1, 1, CV_32FC1 );
    CvMat*y_measurement=cvCreateMat( 1, 1, CV_32FC1 );

    double x_estimated_last=0, y_estimated_last=0;

    static bool initialise=true;
    static FILE*fp;
    if(initialise)
    {
        fp=fopen("data","w");
        if(!fp)
        {
            perror("Open data");
            printf("Cannot save objects data to disk\nPlease do not chose an object it will cause a segmentation fault\n");
        }

        initialise=false;
    }

    for(uint i=0;i<list.size();i++)
    {

        ///If the object is new, we set the prestate and poststate, given that this is not a new measurement, of the filter to the current measurement
        if(list[i]->timers.lifetime==0)
        {
            cvSetReal2D(list[i]->motion_models.cv_x_kalman->state_pre,0,0,list[i]->measurements.x);
            cvSetReal2D(list[i]->motion_models.cv_y_kalman->state_pre,0,0,list[i]->measurements.y);

            cvSetReal2D(list[i]->motion_models.ca_x_kalman->state_pre,0,0,list[i]->measurements.x);
            cvSetReal2D(list[i]->motion_models.ca_y_kalman->state_pre,0,0,list[i]->measurements.y);

            cvSetReal2D(list[i]->motion_models.cv_x_kalman->state_post,0,0,list[i]->measurements.x);
            cvSetReal2D(list[i]->motion_models.cv_y_kalman->state_post,0,0,list[i]->measurements.y);

            cvSetReal2D(list[i]->motion_models.ca_x_kalman->state_post,0,0,list[i]->measurements.x);
            cvSetReal2D(list[i]->motion_models.ca_y_kalman->state_post,0,0,list[i]->measurements.y);
        }else
        {
            ///Get the new measurement into the filter and correct, t=k

            cvSetReal2D(x_measurement,0,0,list[i]->measurements.x);
            cvSetReal2D(y_measurement,0,0,list[i]->measurements.y);

            x_estimated_last=list[i]->position.estimated_x;
            y_estimated_last=list[i]->position.estimated_y;

            cvKalmanCorrect(list[i]->motion_models.cv_x_kalman,x_measurement);
            cvKalmanCorrect(list[i]->motion_models.cv_y_kalman,y_measurement);

            cvKalmanCorrect(list[i]->motion_models.ca_x_kalman,x_measurement);
            cvKalmanCorrect(list[i]->motion_models.ca_y_kalman,y_measurement);
        }

        if((int)list[i]->id==select_object)
            fprintf(fp,"%2.6f %2.6f ",list[i]->measurements.x,list[i]->measurements.y);

        ///Extract the correct state from the filter and put it to path[k]

        double cv_pestimated_x=cvGetReal2D(list[i]->motion_models.cv_x_kalman->state_post,0,0);
        double cv_pestimated_y=cvGetReal2D(list[i]->motion_models.cv_y_kalman->state_post,0,0);

        double ca_pestimated_x=cvGetReal2D(list[i]->motion_models.ca_x_kalman->state_post,0,0);
        double ca_pestimated_y=cvGetReal2D(list[i]->motion_models.ca_y_kalman->state_post,0,0);

        if((int)list[i]->id==select_object)
            fprintf(fp,"%2.6f %2.6f %2.6f %2.6f ",cv_pestimated_x,cv_pestimated_y,ca_pestimated_x,ca_pestimated_y);

        ///After correction iterate the filter to make a prediction, t=k+1
        cvKalmanPredict(list[i]->motion_models.cv_x_kalman);
        cvKalmanPredict(list[i]->motion_models.cv_y_kalman);

        cvKalmanPredict(list[i]->motion_models.ca_x_kalman);
        cvKalmanPredict(list[i]->motion_models.ca_y_kalman);

        ///Extract the prediction from the filter

        double cv_ppredicted_x=cvGetReal2D(list[i]->motion_models.cv_x_kalman->state_pre,0,0);
        double cv_ppredicted_y=cvGetReal2D(list[i]->motion_models.cv_y_kalman->state_pre,0,0);

        double ca_ppredicted_x=cvGetReal2D(list[i]->motion_models.ca_x_kalman->state_pre,0,0);
        double ca_ppredicted_y=cvGetReal2D(list[i]->motion_models.ca_y_kalman->state_pre,0,0);

        if((int)list[i]->id==select_object)
            fprintf(fp,"%2.6f %2.6f %2.6f %2.6f ",cv_ppredicted_x,cv_ppredicted_y,ca_ppredicted_x,ca_ppredicted_y);

        // 		CvPoint a=cvPoint(real2print(x_ca_pre,config),real2print(y_ca_pre,config));

        // 		cvLine(img,a,a,CV_RGB(0,255,0),2, 8, 0 );

        ///Obtain error vectors

        double cv_inno_x=list[i]->measurements.x-cv_ppredicted_x;
        double cv_inno_y=list[i]->measurements.y-cv_ppredicted_y;

        double ca_inno_x=list[i]->measurements.x-ca_ppredicted_x;
        double ca_inno_y=list[i]->measurements.y-ca_ppredicted_y;

        double cv_resi_x=list[i]->measurements.x-cv_pestimated_x;
        double cv_resi_y=list[i]->measurements.y-cv_pestimated_y;

        double ca_resi_x=list[i]->measurements.x-ca_pestimated_x;
        double ca_resi_y=list[i]->measurements.y-ca_pestimated_y;

        // 		double scvi=sqrt(pow(cv_inno_x,2)+pow(cv_inno_y,2));
        // 		double scai=sqrt(pow(ca_inno_x,2)+pow(ca_inno_y,2));

        if((int)list[i]->id==select_object)
            fprintf(fp,"%2.6f %2.6f %2.6f %2.6f ",cv_inno_x,cv_inno_y,ca_inno_x,ca_inno_y);

        if((int)list[i]->id==select_object)
            fprintf(fp,"%2.6f %2.6f %2.6f %2.6f ",cv_resi_x,cv_resi_y,ca_resi_x,ca_resi_y);

        // 		if(scvi<scai)
        // 			list[i]->model=CV;
        // 		else

        // 		list[i]->model=CA;
        // 		if(list[i]->timers.lifetime<30)
        // 			list[i]->model=CV;
        // //
        // 			list[i]->model=MIX;

        // 		if(list[i]->classification.occluded)
        // 			list[i]->model=CV;

        // 		list[i]->model=CA;
        list[i]->model=CV;

        switch(list[i]->model)
        {
            case CV:
                list[i]->position.estimated_x=cv_pestimated_x;
                list[i]->position.estimated_y=cv_pestimated_y;
                list[i]->position.predicted_x=cv_ppredicted_x;
                list[i]->position.predicted_y=cv_ppredicted_y;
                break;
            case CA:
                list[i]->position.estimated_x=ca_pestimated_x;
                list[i]->position.estimated_y=ca_pestimated_y;
                list[i]->position.predicted_x=ca_ppredicted_x;
                list[i]->position.predicted_y=ca_ppredicted_y;
                break;
            case MIX:
                list[i]->position.estimated_x=(ca_pestimated_x+cv_pestimated_x)/2;
                list[i]->position.estimated_y=(ca_pestimated_y+cv_pestimated_y)/2;
                list[i]->position.predicted_x=(ca_ppredicted_x+cv_ppredicted_x)/2;
                list[i]->position.predicted_y=(ca_ppredicted_y+cv_ppredicted_y)/2;
                break;
        }

        ///Add estimated point to path
        AddPointPath(&(list[i]->path_cv),list[i]->position.estimated_x,list[i]->position.estimated_y);
        AddPointPath(&(list[i]->path_ca),list[i]->position.estimated_x,list[i]->position.estimated_y);

        ///Put velocity into data file
        if((int)list[i]->id==select_object)
        {
            double cvvx,cvvy,cavx,cavy;
            cvvx=cvGetReal2D(list[i]->motion_models.cv_x_kalman->state_post,1,0);
            cvvy=cvGetReal2D(list[i]->motion_models.cv_y_kalman->state_post,1,0);
            cavx=cvGetReal2D(list[i]->motion_models.ca_x_kalman->state_post,1,0);
            cavy=cvGetReal2D(list[i]->motion_models.ca_y_kalman->state_post,1,0);

            fprintf(fp,"%2.6f %2.6f %2.6f %2.6f ",cvvx,cvvy,cavx,cavy);
        }

        ///Obtain velocity

        double lateral_error;

        switch(list[i]->model)
        {
            case CV:
                list[i]->velocity.velocity_x=cvGetReal2D(list[i]->motion_models.cv_x_kalman->state_post,1,0);
                list[i]->velocity.velocity_y=cvGetReal2D(list[i]->motion_models.cv_y_kalman->state_post,1,0);
                break;
            case CA:
                list[i]->velocity.velocity_x=cvGetReal2D(list[i]->motion_models.ca_x_kalman->state_post,1,0);
                list[i]->velocity.velocity_y=cvGetReal2D(list[i]->motion_models.ca_y_kalman->state_post,1,0);
                break;
            default:
                list[i]->velocity.velocity_x=cvGetReal2D(list[i]->motion_models.cv_x_kalman->state_post,1,0);
                list[i]->velocity.velocity_y=cvGetReal2D(list[i]->motion_models.cv_y_kalman->state_post,1,0);
                break;
        }

        list[i]->velocity.velocity_module=sqrt(pow(list[i]->velocity.velocity_x,2)+pow(list[i]->velocity.velocity_y,2));
        list[i]->velocity.velocity_angle=atan2(-list[i]->velocity.velocity_y,list[i]->velocity.velocity_x);

        double xi,xf,yi,yf;

        xi=x_estimated_last;
        yi=y_estimated_last;
        xf=xi+list[i]->velocity.velocity_x;
        yf=yi+list[i]->velocity.velocity_y;

        double alpha=atan2(xi-xf,yf-yi)+M_PI;
        double ro=xi*cos(alpha)+yi*sin(alpha);

        double x_m=cvGetReal2D(x_measurement,0,0);
        double y_m=cvGetReal2D(y_measurement,0,0);

        lateral_error = point2line_distance(alpha,ro,x_m,y_m);

        AddPointErrorVectors(&(list[i]->errors_cv),cv_inno_x,cv_inno_y,cv_resi_x,cv_resi_y,lateral_error);
        AddPointErrorVectors(&(list[i]->errors_ca),ca_inno_x,ca_inno_y,ca_resi_x,ca_resi_y,lateral_error);

        ///Put lateral error into data file
        if((int)list[i]->id==select_object)
            fprintf(fp,"%2.6f ",lateral_error);

        ///Put error cov and AKF gains into data file
        if((int)list[i]->id==select_object)
        {
            double cvcovx,cvcovy,cacovx,cacovy;
            cvcovx=list[i]->errors_cv.x_inno_cov;
            cvcovy=list[i]->errors_cv.y_inno_cov;
            cacovx=list[i]->errors_ca.x_inno_cov;
            cacovy=list[i]->errors_ca.y_inno_cov;

            fprintf(fp,"%2.6f %2.6f %2.6f %2.6f ",cvcovx,cvcovy,cacovx,cacovy);

            double cvgpx,cvgpy,cvgvx,cvgvy,cagpx,cagpy,cagvx,cagvy;
            cvgpx=cvGetReal2D(list[i]->motion_models.cv_x_kalman->gain,0,0);
            cvgpy=cvGetReal2D(list[i]->motion_models.cv_y_kalman->gain,0,0);
            cvgvx=cvGetReal2D(list[i]->motion_models.cv_x_kalman->gain,1,0);
            cvgvy=cvGetReal2D(list[i]->motion_models.cv_y_kalman->gain,1,0);

            cagpx=cvGetReal2D(list[i]->motion_models.ca_x_kalman->gain,0,0);
            cagpy=cvGetReal2D(list[i]->motion_models.ca_y_kalman->gain,0,0);
            cagvx=cvGetReal2D(list[i]->motion_models.ca_x_kalman->gain,1,0);
            cagvy=cvGetReal2D(list[i]->motion_models.ca_y_kalman->gain,1,0);

            fprintf(fp,"%2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f %2.6f ",cvgpx,cvgpy,cvgvx,cvgvy,cagpx,cagpy,cagvx,cagvy);
        }

        double previous_factor=1;

        if(list[i]->classification.velocity_classification==STATIONARY)
            previous_factor=2;
        else if(list[i]->classification.velocity_classification==MOVING)
            previous_factor=1./2.;

        if(list[i]->velocity.velocity_module < config.max_stationary_velocity*previous_factor)
        {
            list[i]->classification.velocity_classification=STATIONARY;
            // 			printf("Id %d Stationary\n",objects[i]->id);
        }
        if(list[i]->velocity.velocity_module > config.min_moving_velocity*previous_factor || list[i]->timers.lifetime<5)
        {
            list[i]->classification.velocity_classification=MOVING;
            // 			printf("Id %d Moving\n",objects[i]->id);
        }

        ///Define a new search area based on the kalman errors and predicted position

        SetSearchArea(*list[i],config);

        ///Do some tweaks on the kalman errors

        if(list[i]->classification.velocity_classification==STATIONARY)
        {
            cvSetIdentity( list[i]->motion_models.cv_x_kalman->measurement_noise_cov, cvRealScalar(0.1*0.1));
            cvSetIdentity( list[i]->motion_models.cv_y_kalman->measurement_noise_cov, cvRealScalar(0.1*0.1));

            cvSetIdentity( list[i]->motion_models.ca_x_kalman->measurement_noise_cov, cvRealScalar(0.1*0.1));
            cvSetIdentity( list[i]->motion_models.ca_y_kalman->measurement_noise_cov, cvRealScalar(0.1*0.1));
        }else
        {
            cvSetIdentity( list[i]->motion_models.cv_x_kalman->measurement_noise_cov, cvRealScalar(0.05*0.05));
            cvSetIdentity( list[i]->motion_models.cv_y_kalman->measurement_noise_cov, cvRealScalar(0.05*0.05));

            cvSetIdentity( list[i]->motion_models.ca_x_kalman->measurement_noise_cov, cvRealScalar(0.05*0.05));
            cvSetIdentity( list[i]->motion_models.ca_y_kalman->measurement_noise_cov, cvRealScalar(0.05*0.05));
        }

        ///* Update white noise scale **************************/
        double xAcv=sqrt(list[i]->errors_cv.x_inno_cov);
        double yAcv=sqrt(list[i]->errors_cv.y_inno_cov);
        double xAca=sqrt(list[i]->errors_ca.x_inno_cov);
        double yAca=sqrt(list[i]->errors_ca.y_inno_cov);

        if(xAcv<0.001)xAcv=0.001;
        if(yAcv<0.001)yAcv=0.001;

        if(xAcv>2)xAcv=2;
        if(yAcv>2)yAcv=2;

        if(xAca<0.001)xAca=0.001;
        if(yAca<0.001)yAca=0.001;

        if(xAca>2)xAca=2;
        if(yAca>2)yAca=2;

        double dt=config.dt;
        // 			printf("Id %d xA %2.2f yA %2.2f\n",objects[i]->id,xA,yA);

        if(list[i]->classification.occluded==false)///Only update if the object is visible
        {
            double xQcv[]={pow(xAcv,2)*pow(dt,3)/3,pow(xAcv,2)*pow(dt,2)/2,pow(xAcv,2)*pow(dt,2)/2,pow(xAcv,2)*pow(dt,1)};
            double yQcv[]={pow(yAcv,2)*pow(dt,3)/3,pow(yAcv,2)*pow(dt,2)/2,pow(yAcv,2)*pow(dt,2)/2,pow(yAcv,2)*pow(dt,1)};

            double xQca[]={pow(xAca,2)*pow(dt,5)/20, pow(xAca,2)*pow(dt,4)/8, pow(xAca,2)*pow(dt,3)/6,
                           pow(xAca,2)*pow(dt,4)/8, pow(xAca,2)*pow(dt,3)/3,  pow(xAca,2)*pow(dt,2)/2,
                           pow(xAca,2)*pow(dt,3)/6,  pow(xAca,2)*pow(dt,2)/2,  pow(xAca,2)*pow(dt,1)/1};

            double yQca[]={pow(yAca,2)*pow(dt,5)/20, pow(yAca,2)*pow(dt,4)/8, pow(yAca,2)*pow(dt,3)/6,
                           pow(yAca,2)*pow(dt,4)/8, pow(yAca,2)*pow(dt,3)/3,  pow(yAca,2)*pow(dt,2)/2,
                           pow(yAca,2)*pow(dt,3)/6,  pow(yAca,2)*pow(dt,2)/2,  pow(yAca,2)*pow(dt,1)/1};

            cvSetReal2D(list[i]->motion_models.cv_x_kalman->process_noise_cov, 0,0,xQcv[0] );
            cvSetReal2D(list[i]->motion_models.cv_x_kalman->process_noise_cov, 0,1,xQcv[1] );
            cvSetReal2D(list[i]->motion_models.cv_x_kalman->process_noise_cov, 1,0,xQcv[2] );
            cvSetReal2D(list[i]->motion_models.cv_x_kalman->process_noise_cov, 1,1,xQcv[3] );

            cvSetReal2D(list[i]->motion_models.cv_y_kalman->process_noise_cov, 0,0,yQcv[0] );
            cvSetReal2D(list[i]->motion_models.cv_y_kalman->process_noise_cov, 0,1,yQcv[1] );
            cvSetReal2D(list[i]->motion_models.cv_y_kalman->process_noise_cov, 1,0,yQcv[2] );
            cvSetReal2D(list[i]->motion_models.cv_y_kalman->process_noise_cov, 1,1,yQcv[3] );

            cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 0,0,xQca[0] );
            cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 0,1,xQca[1] );
            cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 0,2,xQca[2] );
            cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 1,0,xQca[3] );
            cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 1,1,xQca[4] );
            cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 1,2,xQca[5] );
            cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 2,0,xQca[6] );
            cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 2,1,xQca[7] );
            cvSetReal2D(list[i]->motion_models.ca_x_kalman->process_noise_cov, 2,2,xQca[8] );

            cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 0,0,yQca[0] );
            cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 0,1,yQca[1] );
            cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 0,2,yQca[2] );
            cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 1,0,yQca[3] );
            cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 1,1,yQca[4] );
            cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 1,2,yQca[5] );
            cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 2,0,yQca[6] );
            cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 2,1,yQca[7] );
            cvSetReal2D(list[i]->motion_models.ca_y_kalman->process_noise_cov, 2,2,yQca[8] );
        }

        if((int)list[i]->id==select_object)
            fprintf(fp,"\n");
    }

    cvReleaseMat(&(x_measurement));
    cvReleaseMat(&(y_measurement));
}

void AllocMotionModels(t_list&list,t_config&config)
{
    list.motion_models.cv_x_kalman = cvCreateKalman( 2, 1, 0 );
    list.motion_models.cv_y_kalman = cvCreateKalman( 2, 1, 0 );

    list.motion_models.ca_x_kalman = cvCreateKalman( 3, 1, 0 );
    list.motion_models.ca_y_kalman = cvCreateKalman( 3, 1, 0 );

    float a=20;
    float dt=config.dt;

    float Acv[] = { 1, (float)dt,
                    0, 1 };

    float dt_2 = dt*dt;
    float Aca[] = { 1,    dt,  (float)(0.5)*dt_2,
                    0,     1,             dt,
                    0,     0,              1.0 };

    double Qcv[]={a*a*pow(dt,3)/3,  a*a*pow(dt,2)/2,
                  a*a*pow(dt,2)/2,  a*a*pow(dt,1)};

    double Qca[]={pow(a,2)*pow(dt,5)/20, pow(a,2)*pow(dt,4)/8, pow(a,2)*pow(dt,3)/6,
                  pow(a,2)*pow(dt,4)/8, pow(a,2)*pow(dt,3)/3,  pow(a,2)*pow(dt,2)/2,
                  pow(a,2)*pow(dt,3)/6,  pow(a,2)*pow(dt,2)/2,  pow(a,2)*pow(dt,1)/1};

    memcpy(list.motion_models.cv_x_kalman->transition_matrix->data.fl, Acv, sizeof(Acv));
    memcpy(list.motion_models.cv_y_kalman->transition_matrix->data.fl, Acv, sizeof(Acv));

    memcpy(list.motion_models.ca_x_kalman->transition_matrix->data.fl, Aca, sizeof(Aca));
    memcpy(list.motion_models.ca_y_kalman->transition_matrix->data.fl, Aca, sizeof(Aca));

    cvSetIdentity( list.motion_models.cv_x_kalman->measurement_matrix, cvRealScalar(1) );
    cvSetIdentity( list.motion_models.cv_y_kalman->measurement_matrix, cvRealScalar(1) );

    cvSetIdentity( list.motion_models.ca_x_kalman->measurement_matrix, cvRealScalar(1) );
    cvSetIdentity( list.motion_models.ca_y_kalman->measurement_matrix, cvRealScalar(1) );

    cvSetIdentity( list.motion_models.cv_x_kalman->measurement_noise_cov, cvRealScalar(0.04*0.04));
    cvSetIdentity( list.motion_models.cv_x_kalman->error_cov_post, cvRealScalar(1));

    cvSetIdentity( list.motion_models.ca_x_kalman->measurement_noise_cov, cvRealScalar(0.04*0.04));
    cvSetIdentity( list.motion_models.ca_x_kalman->error_cov_post, cvRealScalar(1));

    cvSetIdentity( list.motion_models.cv_y_kalman->measurement_noise_cov, cvRealScalar(0.04*0.04));
    cvSetIdentity( list.motion_models.cv_y_kalman->error_cov_post, cvRealScalar(1));

    cvSetIdentity( list.motion_models.ca_y_kalman->measurement_noise_cov, cvRealScalar(0.04*0.04));
    cvSetIdentity( list.motion_models.ca_y_kalman->error_cov_post, cvRealScalar(1));

    cvSetReal2D(list.motion_models.cv_x_kalman->process_noise_cov, 0,0,Qcv[0] );
    cvSetReal2D(list.motion_models.cv_x_kalman->process_noise_cov, 0,1,Qcv[1] );
    cvSetReal2D(list.motion_models.cv_x_kalman->process_noise_cov, 1,0,Qcv[2] );
    cvSetReal2D(list.motion_models.cv_x_kalman->process_noise_cov, 1,1,Qcv[3] );

    cvSetReal2D(list.motion_models.cv_y_kalman->process_noise_cov, 0,0,Qcv[0] );
    cvSetReal2D(list.motion_models.cv_y_kalman->process_noise_cov, 0,1,Qcv[1] );
    cvSetReal2D(list.motion_models.cv_y_kalman->process_noise_cov, 1,0,Qcv[2] );
    cvSetReal2D(list.motion_models.cv_y_kalman->process_noise_cov, 1,1,Qcv[3] );

    cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 0,0,Qca[0] );
    cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 0,1,Qca[1] );
    cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 0,2,Qca[2] );
    cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 1,0,Qca[3] );
    cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 1,1,Qca[4] );
    cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 1,2,Qca[5] );
    cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 2,0,Qca[6] );
    cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 2,1,Qca[7] );
    cvSetReal2D(list.motion_models.ca_x_kalman->process_noise_cov, 2,2,Qca[8] );

    cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 0,0,Qca[0] );
    cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 0,1,Qca[1] );
    cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 0,2,Qca[2] );
    cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 1,0,Qca[3] );
    cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 1,1,Qca[4] );
    cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 1,2,Qca[5] );
    cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 2,0,Qca[6] );
    cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 2,1,Qca[7] );
    cvSetReal2D(list.motion_models.ca_y_kalman->process_noise_cov, 2,2,Qca[8] );

    cvZero(list.motion_models.cv_x_kalman->state_post);
    cvZero(list.motion_models.cv_y_kalman->state_post);

    cvZero(list.motion_models.ca_x_kalman->state_post);
    cvZero(list.motion_models.ca_y_kalman->state_post);

    return;
}

/** @fn void GetErrorConvariance(t_errors*error)
* @brief Get covariance
* @return void
*/
void GetErrorConvariance(t_errors*error)
{
    ///Initalise auxiliar matrixes

    CvMat*xi_error_element = cvCreateMat(2,2,CV_64FC1);
    CvMat*xr_error_element = cvCreateMat(2,2,CV_64FC1);
    CvMat*yi_error_element = cvCreateMat(2,2,CV_64FC1);
    CvMat*yr_error_element = cvCreateMat(2,2,CV_64FC1);
    CvMat*lateral_error_element = cvCreateMat(2,2,CV_64FC1);

    CvMat*xi_element = cvCreateMat(2,1,CV_64FC1);
    CvMat*xr_element = cvCreateMat(2,1,CV_64FC1);
    CvMat*yi_element = cvCreateMat(2,1,CV_64FC1);
    CvMat*yr_element = cvCreateMat(2,1,CV_64FC1);
    CvMat*lateral_element = cvCreateMat(2,1,CV_64FC1);

    CvMat*xi_element_t = cvCreateMat(1,2,CV_64FC1);
    CvMat*xr_element_t = cvCreateMat(1,2,CV_64FC1);
    CvMat*yi_element_t = cvCreateMat(1,2,CV_64FC1);
    CvMat*yr_element_t = cvCreateMat(1,2,CV_64FC1);
    CvMat*lateral_element_t = cvCreateMat(1,2,CV_64FC1);

    CvMat*xi_error_accumulator = cvCreateMat(2,2,CV_64FC1);
    CvMat*xr_error_accumulator = cvCreateMat(2,2,CV_64FC1);
    CvMat*yi_error_accumulator = cvCreateMat(2,2,CV_64FC1);
    CvMat*yr_error_accumulator = cvCreateMat(2,2,CV_64FC1);
    CvMat*lateral_error_accumulator = cvCreateMat(2,2,CV_64FC1);

    CvMat*ones = cvCreateMat(2,2,CV_64FC1);

    double multiplier=1;

    ///Set accumulators to 0

    cvSet(xi_error_accumulator,cvScalar(0),NULL);
    cvSet(xr_error_accumulator,cvScalar(0),NULL);
    cvSet(yi_error_accumulator,cvScalar(0),NULL);
    cvSet(yr_error_accumulator,cvScalar(0),NULL);
    cvSet(lateral_error_accumulator,cvScalar(0),NULL);

    ///Do Cov(Innovation) = 1/m * S(i=0,i< m-1,d[k-i]*d[k-i]') and Cov(Residue) = 1/m * S(i=0,i< m-1,e[k-i]*e[k-i]')
    for(unsigned int i=0;i<error->number_points;i++)
    {
        // 		printf("x_cv_innovation %2.2f\n",error->x_cv_innovation[i]);
        cvmSet(xi_element,0,0,error->x_innovation[i]);
        cvmSet(xi_element,1,0,0);

        cvmSet(xr_element,0,0,error->x_residue[i]);
        cvmSet(xr_element,1,0,0);

        cvmSet(yi_element,0,0,error->y_innovation[i]);
        cvmSet(yi_element,1,0,0);

        cvmSet(yr_element,0,0,error->y_residue[i]);
        cvmSet(yr_element,1,0,0);

        cvmSet(lateral_element,0,0,error->lateral_error[i]);
        cvmSet(lateral_element,1,0,0);

        cvTranspose(xi_element,xi_element_t);
        cvTranspose(xr_element,xr_element_t);
        cvTranspose(yi_element,yi_element_t);
        cvTranspose(yr_element,yr_element_t);
        cvTranspose(lateral_element,lateral_element_t);

        cvMatMul(xi_element,xi_element_t,xi_error_element);
        cvMatMul(xr_element,xr_element_t,xr_error_element);
        cvMatMul(yi_element,yi_element_t,yi_error_element);
        cvMatMul(yr_element,yr_element_t,yr_error_element);
        cvMatMul(lateral_element,lateral_element_t,lateral_error_element);

        cvAdd(xi_error_element,xi_error_accumulator,xi_error_accumulator,NULL);
        cvAdd(xr_error_element,xr_error_accumulator,xr_error_accumulator,NULL);
        cvAdd(yi_error_element,yi_error_accumulator,yi_error_accumulator,NULL);
        cvAdd(yr_error_element,yr_error_accumulator,yr_error_accumulator,NULL);
        cvAdd(lateral_error_element,lateral_error_accumulator,lateral_error_accumulator,NULL);
    }

    if(error->number_points==0)
        goto gecEND;

    cvSet(ones,cvScalar(1));
    multiplier=1./(double)error->number_points;
    //printf("Muli %2.8f\n",multiplier);
    ///@todo Wend the number of points is growing the covariance is way to big, i don't realy know why

    cvMul(xi_error_accumulator,ones,xi_error_accumulator,multiplier);
    cvMul(xr_error_accumulator,ones,xr_error_accumulator,multiplier);
    cvMul(yi_error_accumulator,ones,yi_error_accumulator,multiplier);
    cvMul(yr_error_accumulator,ones,yr_error_accumulator,multiplier);
    cvMul(lateral_error_accumulator,ones,lateral_error_accumulator,multiplier);

    error->x_inno_cov = cvGetReal2D(xi_error_accumulator,0,0);
    error->x_resi_cov = cvGetReal2D(xr_error_accumulator,0,0);
    error->y_inno_cov = cvGetReal2D(yi_error_accumulator,0,0);
    error->y_resi_cov = cvGetReal2D(yr_error_accumulator,0,0);
    error->lateral_error_cov = cvGetReal2D(lateral_error_accumulator,0,0);

    gecEND:

    cvReleaseMat(&(xi_error_element));
    cvReleaseMat(&(xr_error_element));
    cvReleaseMat(&(yi_error_element));
    cvReleaseMat(&(yr_error_element));
    cvReleaseMat(&(lateral_error_element));

    cvReleaseMat(&(xi_element));
    cvReleaseMat(&(xr_element));
    cvReleaseMat(&(yi_element));
    cvReleaseMat(&(yr_element));
    cvReleaseMat(&(lateral_element));

    cvReleaseMat(&(xi_element_t));
    cvReleaseMat(&(xr_element_t));
    cvReleaseMat(&(yi_element_t));
    cvReleaseMat(&(yr_element_t));
    cvReleaseMat(&(lateral_element_t));

    cvReleaseMat(&(xi_error_accumulator));
    cvReleaseMat(&(xr_error_accumulator));
    cvReleaseMat(&(yi_error_accumulator));
    cvReleaseMat(&(yr_error_accumulator));
    cvReleaseMat(&(lateral_error_accumulator));
    cvReleaseMat(&(ones));

    return;
}



CvKalman*CreateModelCTRV(void)
{
    double vQ=1;
    double vR=10;
    double vP=5;

    CvKalman*model=cvCreateKalman(5,5,0);

    cvSetIdentity(model->measurement_matrix,cvRealScalar(1));
    cvSetReal2D(model->measurement_matrix,0,0,0);
    cvSetReal2D(model->measurement_matrix,1,0,0);
    // 	cvSetReal2D(model->measurement_matrix,2,0,0);
    cvSetReal2D(model->measurement_matrix,4,0,0);

    cvSetIdentity(model->measurement_noise_cov,cvRealScalar(vR));
    // 	cvSetReal2D(model->measurement_noise_cov,3,0,vR);
    // 	cvSetReal2D(model->measurement_noise_cov,4,0,vR);

    cvSetIdentity(model->process_noise_cov,cvRealScalar(vQ));
    // 	cvSetReal2D(model->process_noise_cov,3,0,vQ*0.001);
    // 	cvSetReal2D(model->process_noise_cov,4,0,vQ);

    cvSet(model->state_post,cvRealScalar(0.01));
    cvSetIdentity(model->error_cov_post,cvRealScalar(vP));

    UpdateTransitionMatrixCTRV(model,0,0,0,0,0.01,1./50.);
    /*
    printf("A[0][0] %f\n",cvGetReal2D(model->transition_matrix,0,0));
    printf("A[0][1] %f\n",cvGetReal2D(model->transition_matrix,0,1));
    printf("A[0][2] %f\n",cvGetReal2D(model->transition_matrix,0,2));
    printf("A[0][3] %f\n",cvGetReal2D(model->transition_matrix,0,3));
    printf("A[0][4] %f\n",cvGetReal2D(model->transition_matrix,0,4));
    */
    return model;
}

CvKalman*CreateModelCV_SC(void)
{
    double vR=180;
    double vP=100;

    double dt=1./50.,a=12.;

    CvKalman*model=cvCreateKalman(2,2,0);

    cvSet(model->state_post,cvRealScalar(0));

    cvSetReal2D(model->measurement_matrix,0,0,1);
    cvSetReal2D(model->measurement_matrix,1,0,0);

    cvSetIdentity(model->measurement_noise_cov,cvRealScalar(vR));

    cvSetIdentity(model->error_cov_post,cvRealScalar(vP));

    UpdateTransitionMatrixCV_SC(model,dt);

    cvSetReal2D(model->process_noise_cov,0,0,a*a*pow(dt,3)/3);
    cvSetReal2D(model->process_noise_cov,0,1,a*a*pow(dt,2)/2);
    cvSetReal2D(model->process_noise_cov,1,0,a*a*pow(dt,2)/2);
    cvSetReal2D(model->process_noise_cov,1,1,a*a*pow(dt,1));

    // 	printf("Q:\n");
    // 	printf("%6.2f %6.2f\n",cvGetReal2D(model->process_noise_cov,0,0),cvGetReal2D(model->process_noise_cov,0,1));
    // 	printf("%6.2f %6.2f\n",cvGetReal2D(model->process_noise_cov,1,0),cvGetReal2D(model->process_noise_cov,1,1));

    return model;
}

CvKalman*CreateModelCV(void)
{
    double vR=600;
    double vP=500;

    double dt=1./50.,a=10;

    CvKalman*model=cvCreateKalman(4,4,0);

    cvSet(model->state_post,cvRealScalar(0));

    cvSetReal2D(model->measurement_matrix,0,0,0);
    cvSetReal2D(model->measurement_matrix,1,0,1);
    cvSetReal2D(model->measurement_matrix,2,0,0);
    cvSetReal2D(model->measurement_matrix,3,0,1);

    cvSetIdentity(model->measurement_noise_cov,cvRealScalar(vR));

    cvSetIdentity(model->error_cov_post,cvRealScalar(vP));

    UpdateTransitionMatrixCV(model,dt);

    cvSetReal2D(model->process_noise_cov,0,0,a*a*pow(dt,3)/3);
    cvSetReal2D(model->process_noise_cov,0,1,a*a*pow(dt,2)/2);
    cvSetReal2D(model->process_noise_cov,0,2,0);
    cvSetReal2D(model->process_noise_cov,0,3,0);

    cvSetReal2D(model->process_noise_cov,1,0,a*a*pow(dt,2)/2);
    cvSetReal2D(model->process_noise_cov,1,1,a*a*pow(dt,1));
    cvSetReal2D(model->process_noise_cov,1,2,0);
    cvSetReal2D(model->process_noise_cov,1,3,0);

    cvSetReal2D(model->process_noise_cov,2,0,0);
    cvSetReal2D(model->process_noise_cov,2,1,0);
    cvSetReal2D(model->process_noise_cov,2,2,a*a*pow(dt,3)/3);
    cvSetReal2D(model->process_noise_cov,2,3,a*a*pow(dt,2)/2);

    cvSetReal2D(model->process_noise_cov,3,0,0);
    cvSetReal2D(model->process_noise_cov,3,1,0);
    cvSetReal2D(model->process_noise_cov,3,2,a*a*pow(dt,2)/2);
    cvSetReal2D(model->process_noise_cov,3,3,a*a*pow(dt,1));

    // 	printf("Q:\n");
    // 	printf("%6.2f %6.2f %6.2f %6.2f\n",cvGetReal2D(model->process_noise_cov,0,0),cvGetReal2D(model->process_noise_cov,0,1),cvGetReal2D(model->process_noise_cov,0,2),cvGetReal2D(model->process_noise_cov,0,3));
    // 	printf("%6.2f %6.2f %6.2f %6.2f\n",cvGetReal2D(model->process_noise_cov,1,0),cvGetReal2D(model->process_noise_cov,1,1),cvGetReal2D(model->process_noise_cov,1,2),cvGetReal2D(model->process_noise_cov,1,3));
    // 	printf("%6.2f %6.2f %6.2f %6.2f\n",cvGetReal2D(model->process_noise_cov,2,0),cvGetReal2D(model->process_noise_cov,2,1),cvGetReal2D(model->process_noise_cov,2,2),cvGetReal2D(model->process_noise_cov,2,3));
    // 	printf("%6.2f %6.2f %6.2f %6.2f\n",cvGetReal2D(model->process_noise_cov,3,0),cvGetReal2D(model->process_noise_cov,3,1),cvGetReal2D(model->process_noise_cov,3,2),cvGetReal2D(model->process_noise_cov,3,3));

    return model;
}

CvKalman*CreateModelFwdCt(void)
{
    double vR=500;
    double vP=100;
    double vQ=10;
    double dt=1./50.;

    CvKalman*model=cvCreateKalman(6,2,0);

    dH_FwdCt(model);

    cvSetIdentity(model->measurement_noise_cov,cvRealScalar(vR));

    cvSetIdentity(model->process_noise_cov,cvRealScalar(vQ));
    cvSetIdentity(model->error_cov_post,cvRealScalar(vP));

    double q0[]={0, 0, 0, 0, 0, 0};

    dA_FwdCt(model,q0,dt);

    return model;
}

void IterateMotionModelCTRV(CvKalman*model,double vm,double wm)
{
    double x,y,t,v,w;
    static bool init=true;

    cvKalmanPredict(model);

    CvMat*measurement=cvCreateMat( 5, 1, CV_32FC1 );

    cvZero(measurement);
    cvSetReal2D(measurement,3,0,vm);
    cvSetReal2D(measurement,2,0,wm);

    // 	printf("vM %f wM %f\n",vm,wm);

    x=cvGetReal2D(model->state_post,0,0);
    y=cvGetReal2D(model->state_post,1,0);
    t=cvGetReal2D(model->state_post,2,0);
    v=cvGetReal2D(model->state_post,3,0);
    w=cvGetReal2D(model->state_post,4,0);

    UpdateTransitionMatrixCTRV(model,x,y,t,v,w,1./50.);

    // 	printf("iX %f\n",x);
    // 	printf("iY %f\n",y);
    // 	printf("iT %f\n",t);
    // 	printf("iV %f\n",v);
    // 	printf("iW %f\n",w);

    cvKalmanCorrect(model,measurement);

    if(init)
    {
        cvSet(model->state_post,cvRealScalar(0.01));
        init=false;
    }

    x=cvGetReal2D(model->state_post,0,0);
    y=cvGetReal2D(model->state_post,1,0);
    t=cvGetReal2D(model->state_post,2,0);
    v=cvGetReal2D(model->state_post,3,0);
    w=cvGetReal2D(model->state_post,4,0);

// 	vel=v;
// 	theta=t;

    // 	printf("X %f\n",x);
    // 	printf("Y %f\n",y);
    // 	printf("T %f\n",t);
    // 	printf("V %f\n",v);
    // 	printf("W %f\n",w);



    // 	cvSetReal2D(model->state_post,3,0,0.1);
    // 	cvSetReal2D(model->state_pre,3,0,0.1);

    // 	double cv_pestimated_y=cvGetReal2D(list->motion_models.cv_y_kalman->state_post,0,0);

}


void IterateMotionModelFwdCt(CvKalman*model,double z[2])
{
    // 	static vectorll runtime_PSM;
    static CvMat*measurement=cvCreateMat( 2, 1, CV_32FC1 );
    double q[6];

    q[0]=cvGetReal2D(model->state_post,0,0);
    q[1]=cvGetReal2D(model->state_post,1,0);
    q[2]=cvGetReal2D(model->state_post,2,0);
    q[3]=cvGetReal2D(model->state_post,3,0);
    q[4]=cvGetReal2D(model->state_post,4,0);
    q[5]=cvGetReal2D(model->state_post,5,0);

    cvSetReal2D(measurement,0,0,z[0]);
    cvSetReal2D(measurement,1,0,z[1]);

    dA_FwdCt(model,q,1/50.);//update transition matrix
    cvKalmanCorrect(model,measurement);
    cvKalmanPredict(model);

    q[0]=cvGetReal2D(model->state_post,0,0);
    q[1]=cvGetReal2D(model->state_post,1,0);
    q[2]=cvGetReal2D(model->state_post,2,0);
    q[3]=cvGetReal2D(model->state_post,3,0);
    q[4]=cvGetReal2D(model->state_post,4,0);
    q[5]=cvGetReal2D(model->state_post,5,0);

// 	memcpy(s,q,6*sizeof(double));

    printf("X %6.6f Y %6.6f V1 %6.6f T %6.6f F %6.6f V2 %6.6f\n",q[0],q[1],q[2],q[3],q[4],q[5]);
}

double IterateMotionModelCV_SC(CvKalman*model,double vm)
{
    double x;

    CvMat*measurement=cvCreateMat( 2, 1, CV_32FC1 );

    cvZero(measurement);
    cvSetReal2D(measurement,0,0,vm);

    // 	printf("Vx %f\n",vxm);
    // 	printf("z:\n %6.2f %6.2f\n",cvGetReal2D(measurement,0,0),cvGetReal2D(measurement,1,0));

    // 	printf("H:\n %6.2f %6.2f\n",cvGetReal2D(model->measurement_matrix,0,0),cvGetReal2D(model->measurement_matrix,1,0));

    // 	x=cvGetReal2D(model->state_post,0,0);
    // 	vx=cvGetReal2D(model->state_post,1,0);

    // 	printf("x:\n %6.2f %6.2f\n",x,vx);

    cvKalmanCorrect(model,measurement);

    x=cvGetReal2D(model->state_post,0,0);
    // 	vx=cvGetReal2D(model->state_post,1,0);

    // 	vel=sqrt(pow(vx,2)+pow(vy,2));
    // 	theta=atan2(vy,vx);

    // 	printf("x:\n %6.2f %6.2f\n",x,vx);

    cvKalmanPredict(model);

    // 	double gx=cvGetReal2D(model->gain,0,0);
    // 	double gvx=cvGetReal2D(model->gain,1,1);

    // 	printf("G:\n %6.2f %6.2f\n",gx,gvx);

    return x;
}

void IterateMotionModelCV(CvKalman*model,double vxm,double vym)
{
    CvMat*measurement=cvCreateMat( 4, 1, CV_32FC1 );

    cvZero(measurement);
    cvSetReal2D(measurement,1,0,vxm);
    cvSetReal2D(measurement,3,0,vym);

    // 	printf("Vx %f\n",vxm);
    // 	printf("z:\n %6.2f %6.2f %6.2f %6.2f\n",cvGetReal2D(measurement,0,0),cvGetReal2D(measurement,1,0),cvGetReal2D(measurement,2,0),cvGetReal2D(measurement,3,0));

    // 	printf("H:\n %6.2f %6.2f %6.2f %6.2f\n",cvGetReal2D(model->measurement_matrix,0,0),cvGetReal2D(model->measurement_matrix,1,0),cvGetReal2D(model->measurement_matrix,2,0),cvGetReal2D(model->measurement_matrix,3,0));

// 	x=cvGetReal2D(model->state_post,0,0);
// 	vx=cvGetReal2D(model->state_post,1,0);
// 	y=cvGetReal2D(model->state_post,2,0);
// 	vy=cvGetReal2D(model->state_post,3,0);

    // 	printf("x:\n %6.2f %6.2f %6.2f %6.2f\n",x,vx,y,vy);

    cvKalmanCorrect(model,measurement);

// 	x=cvGetReal2D(model->state_post,0,0);
// 	vx=cvGetReal2D(model->state_post,1,0);
// 	y=cvGetReal2D(model->state_post,2,0);
// 	vy=cvGetReal2D(model->state_post,3,0);

// 	vel=sqrt(pow(vx,2)+pow(vy,2));
// 	theta=atan2(vy,vx);

    // 	printf("x:\n %6.2f %6.2f %6.2f %6.2f\n",x,vx,y,vy);

    cvKalmanPredict(model);

    // 	double gx=cvGetReal2D(model->gain,0,0);
    // 	double gvx=cvGetReal2D(model->gain,1,1);
    // 	double gy=cvGetReal2D(model->gain,2,2);
    // 	double gvy=cvGetReal2D(model->gain,3,3);

    // 	printf("G:\n %6.2f %6.2f %6.2f %6.2f\n",gx,gvx,gy,gvy);
}

void UpdateTransitionMatrixCV_SC(CvKalman*model,double dt)
{
    double tM[2][2];

    tM[0][0]=1;
    tM[0][1]=dt;

    tM[1][0]=0;
    tM[1][1]=1;

    cvSetReal2D(model->transition_matrix, 0,0,tM[0][0]);
    cvSetReal2D(model->transition_matrix, 0,1,tM[0][1]);
    cvSetReal2D(model->transition_matrix, 1,0,tM[1][0]);
    cvSetReal2D(model->transition_matrix, 1,1,tM[1][1]);

    // 	printf("A:\n");
    // 	printf("%6.2f %6.2f\n",tM[0][0],tM[0][1]);
    // 	printf("%6.2f %6.2f\n",tM[1][0],tM[1][1]);
}

void UpdateTransitionMatrixCV(CvKalman*model,double dt)
{
    double tM[4][4];

    tM[0][0]=1;
    tM[0][1]=dt;
    tM[0][2]=0;
    tM[0][3]=0;

    tM[1][0]=0;
    tM[1][1]=1;
    tM[1][2]=0;
    tM[1][3]=0;

    tM[2][0]=0;
    tM[2][1]=0;
    tM[2][2]=1;
    tM[2][3]=dt;

    tM[3][0]=0;
    tM[3][1]=0;
    tM[3][2]=0;
    tM[3][3]=1;

    cvSetReal2D(model->transition_matrix, 0,0,tM[0][0]);
    cvSetReal2D(model->transition_matrix, 0,1,tM[0][1]);
    cvSetReal2D(model->transition_matrix, 0,2,tM[0][2]);
    cvSetReal2D(model->transition_matrix, 0,3,tM[0][3]);

    cvSetReal2D(model->transition_matrix, 1,0,tM[1][0]);
    cvSetReal2D(model->transition_matrix, 1,1,tM[1][1]);
    cvSetReal2D(model->transition_matrix, 1,2,tM[1][2]);
    cvSetReal2D(model->transition_matrix, 1,3,tM[1][3]);

    cvSetReal2D(model->transition_matrix, 2,0,tM[2][0]);
    cvSetReal2D(model->transition_matrix, 2,1,tM[2][1]);
    cvSetReal2D(model->transition_matrix, 2,2,tM[2][2]);
    cvSetReal2D(model->transition_matrix, 2,3,tM[2][3]);

    cvSetReal2D(model->transition_matrix, 3,0,tM[3][0]);
    cvSetReal2D(model->transition_matrix, 3,1,tM[3][1]);
    cvSetReal2D(model->transition_matrix, 3,2,tM[3][2]);
    cvSetReal2D(model->transition_matrix, 3,3,tM[3][3]);

    // 	printf("A:\n");
    // 	printf("%6.2f %6.2f %6.2f %6.2f\n",tM[0][0],tM[0][1],tM[0][2],tM[0][3]);
    // 	printf("%6.2f %6.2f %6.2f %6.2f\n",tM[1][0],tM[1][1],tM[1][2],tM[1][3]);
    // 	printf("%6.2f %6.2f %6.2f %6.2f\n",tM[2][0],tM[2][1],tM[2][2],tM[2][3]);
    // 	printf("%6.2f %6.2f %6.2f %6.2f\n",tM[3][0],tM[3][1],tM[3][2],tM[3][3]);
}

void dH_FwdCt(CvKalman*model)
{
    cvZero(model->measurement_matrix);

    // 	0     0     1     0     0	  0
    //  0     0     0     0     1	  0

    cvSetReal2D(model->measurement_matrix,0,2,1);
    cvSetReal2D(model->measurement_matrix,1,4,1);

}
void dA_FwdCt(CvKalman*model,double q[6],double dt,double l)
{
    //Calculate dA in x[k-1]

    // 	double x=q[0];
    // 	double y=q[1];
    double v1=q[2];
    double t=q[3];
    double f=q[4];
    // 	double v2=q[5];

    double tm[6][6];

    printf("\n>> V1 %f DT %f\n",v1,dt);
    printf("T %f\n",t);
    printf("F %f\n",f);
    printf("\n");

    //dF1
    tm[0][0]=1;
    tm[0][1]=0;
    tm[0][2]=cos(t)*cos(f)*dt;
    tm[0][3]=-sin(t)*cos(f)*v1*dt;
    tm[0][4]=cos(t)*(-sin(f))*v1*dt;
    tm[0][5]=0;

    tm[1][0]=0;
    tm[1][1]=1;
    tm[1][2]=sin(t)*cos(f)*dt;
    tm[1][3]=cos(t)*cos(f)*v1*dt;
    tm[1][4]=sin(t)*(-sin(f))*v1*dt;
    tm[1][5]=0;

    tm[2][0]=0;
    tm[2][1]=0;
    tm[2][2]=1;
    tm[2][3]=0;
    tm[2][4]=0;
    tm[2][5]=0;

    tm[3][0]=0;
    tm[3][1]=0;
    tm[3][2]=sin(f/l)*dt;
    tm[3][3]=1;
    tm[3][4]=cos(f/l)/l*v1*dt;
    tm[3][5]=0;

    tm[4][0]=0;
    tm[4][1]=0;
    tm[4][2]=0;
    tm[4][3]=0;
    tm[4][4]=1;
    tm[4][5]=dt;

    tm[5][0]=0;
    tm[5][1]=0;
    tm[5][2]=0;
    tm[5][3]=0;
    tm[5][4]=0;
    tm[5][5]=1;

    cvSetReal2D(model->transition_matrix, 0,0,tm[0][0]);
    cvSetReal2D(model->transition_matrix, 0,1,tm[0][1]);
    cvSetReal2D(model->transition_matrix, 0,2,tm[0][2]);
    cvSetReal2D(model->transition_matrix, 0,3,tm[0][3]);
    cvSetReal2D(model->transition_matrix, 0,4,tm[0][4]);
    cvSetReal2D(model->transition_matrix, 0,5,tm[0][5]);

    cvSetReal2D(model->transition_matrix, 1,0,tm[1][0]);
    cvSetReal2D(model->transition_matrix, 1,1,tm[1][1]);
    cvSetReal2D(model->transition_matrix, 1,2,tm[1][2]);
    cvSetReal2D(model->transition_matrix, 1,3,tm[1][3]);
    cvSetReal2D(model->transition_matrix, 1,4,tm[1][4]);
    cvSetReal2D(model->transition_matrix, 1,5,tm[1][5]);

    cvSetReal2D(model->transition_matrix, 2,0,tm[2][0]);
    cvSetReal2D(model->transition_matrix, 2,1,tm[2][1]);
    cvSetReal2D(model->transition_matrix, 2,2,tm[2][2]);
    cvSetReal2D(model->transition_matrix, 2,3,tm[2][3]);
    cvSetReal2D(model->transition_matrix, 2,4,tm[2][4]);
    cvSetReal2D(model->transition_matrix, 2,5,tm[2][5]);

    cvSetReal2D(model->transition_matrix, 3,0,tm[3][0]);
    cvSetReal2D(model->transition_matrix, 3,1,tm[3][1]);
    cvSetReal2D(model->transition_matrix, 3,2,tm[3][2]);
    cvSetReal2D(model->transition_matrix, 3,3,tm[3][3]);
    cvSetReal2D(model->transition_matrix, 3,4,tm[3][4]);
    cvSetReal2D(model->transition_matrix, 3,5,tm[3][5]);

    cvSetReal2D(model->transition_matrix, 4,0,tm[4][0]);
    cvSetReal2D(model->transition_matrix, 4,1,tm[4][1]);
    cvSetReal2D(model->transition_matrix, 4,2,tm[4][2]);
    cvSetReal2D(model->transition_matrix, 4,3,tm[4][3]);
    cvSetReal2D(model->transition_matrix, 4,4,tm[4][4]);
    cvSetReal2D(model->transition_matrix, 4,5,tm[4][5]);

    cvSetReal2D(model->transition_matrix, 5,0,tm[5][0]);
    cvSetReal2D(model->transition_matrix, 5,1,tm[5][1]);
    cvSetReal2D(model->transition_matrix, 5,2,tm[5][2]);
    cvSetReal2D(model->transition_matrix, 5,3,tm[5][3]);
    cvSetReal2D(model->transition_matrix, 5,4,tm[5][4]);
    cvSetReal2D(model->transition_matrix, 5,5,tm[5][5]);

    printf("dA:\n");
    printf("%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n",tm[0][0],tm[0][1],tm[0][2],tm[0][3],tm[0][4],tm[0][5]);
    printf("%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n",tm[1][0],tm[1][1],tm[1][2],tm[1][3],tm[1][4],tm[1][5]);
    printf("%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n",tm[2][0],tm[2][1],tm[2][2],tm[2][3],tm[2][4],tm[2][5]);
    printf("%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n",tm[3][0],tm[3][1],tm[3][2],tm[3][3],tm[3][4],tm[3][5]);
    printf("%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n",tm[4][0],tm[4][1],tm[4][2],tm[4][3],tm[4][4],tm[4][5]);
    printf("%6.6f %6.6f %6.6f %6.6f %6.6f %6.6f\n",tm[5][0],tm[5][1],tm[5][2],tm[5][3],tm[5][4],tm[5][5]);

    // [                    1,                    0,     cos(t)*cos(f)*dt, -sin(t)*cos(f)*v1*dt, -cos(t)*sin(f)*v1*dt,                    0]
    // [                    0,                    1,     sin(t)*cos(f)*dt,  cos(t)*cos(f)*v1*dt, -sin(t)*sin(f)*v1*dt,                    0]
    // [                    0,                    0,                    1,                    0,                    0,                    0]
    // [                    0,                    0,          sin(f/l)*dt,                    1,     cos(f/l)/l*v1*dt,                    0]
    // [                    0,                    0,                    0,                    0,                    1,                   dt]
    // [                    0,                    0,                    0,                    0,                    0,                    1]
}

void UpdateTransitionMatrixCTRV(CvKalman*model,double /*x*/,double /*y*/,double t,double v,double w,double dt)
{
    double tM[5][5];

    tM[0][0]=1;
    tM[0][1]=0;
    tM[0][2]=(v/w)*cos(w*dt+t)-(v/w)*cos(t);
    tM[0][3]=(1/w)*sin(w*dt+t)-(1/w)*sin(t);
    tM[0][4]=-(v/pow(w,2))*sin(w*dt+t)+(v/w)*cos(w*dt+t)*dt+(v/pow(w,2))*sin(t);

    tM[1][0]=0;
    tM[1][1]=1;
    tM[1][2]=(v/w)*sin(w*dt+t)+(v/w)*cos(t);
    tM[1][3]=-(1/w)*cos(w*dt+t)+(1/w)*sin(t);
    tM[1][4]=(v/pow(w,2))*cos(w*dt*t)+(v/w)*sin(w*dt+t)*dt-(v/pow(w,2))*sin(t);

    tM[2][0]=0;
    tM[2][1]=0;
    tM[2][2]=1;
    tM[2][3]=0;
    tM[2][4]=dt;
    // 	printf("DT %f\n",dt);

    tM[3][0]=0;
    tM[3][1]=0;
    tM[3][2]=0;
    tM[3][3]=1;
    tM[3][4]=0;

    tM[4][0]=0;
    tM[4][1]=0;
    tM[4][2]=0;
    tM[4][3]=0;
    tM[4][4]=1;


    cvSetReal2D(model->transition_matrix, 0,0,tM[0][0]);
    cvSetReal2D(model->transition_matrix, 0,1,tM[0][1]);
    cvSetReal2D(model->transition_matrix, 0,2,tM[0][2]);
    cvSetReal2D(model->transition_matrix, 0,3,tM[0][3]);
    cvSetReal2D(model->transition_matrix, 0,4,tM[0][4]);

    cvSetReal2D(model->transition_matrix, 1,0,tM[1][0]);
    cvSetReal2D(model->transition_matrix, 1,1,tM[1][1]);
    cvSetReal2D(model->transition_matrix, 1,2,tM[1][2]);
    cvSetReal2D(model->transition_matrix, 1,3,tM[1][3]);
    cvSetReal2D(model->transition_matrix, 1,4,tM[1][4]);

    cvSetReal2D(model->transition_matrix, 2,0,tM[2][0]);
    cvSetReal2D(model->transition_matrix, 2,1,tM[2][1]);
    cvSetReal2D(model->transition_matrix, 2,2,tM[2][2]);
    cvSetReal2D(model->transition_matrix, 2,3,tM[2][3]);
    cvSetReal2D(model->transition_matrix, 2,4,tM[2][4]);

    cvSetReal2D(model->transition_matrix, 3,0,tM[3][0]);
    cvSetReal2D(model->transition_matrix, 3,1,tM[3][1]);
    cvSetReal2D(model->transition_matrix, 3,2,tM[3][2]);
    cvSetReal2D(model->transition_matrix, 3,3,tM[3][3]);
    cvSetReal2D(model->transition_matrix, 3,4,tM[3][4]);

    cvSetReal2D(model->transition_matrix, 4,0,tM[4][0]);
    cvSetReal2D(model->transition_matrix, 4,1,tM[4][1]);
    cvSetReal2D(model->transition_matrix, 4,2,tM[4][2]);
    cvSetReal2D(model->transition_matrix, 4,3,tM[4][3]);
    cvSetReal2D(model->transition_matrix, 4,4,tM[4][4]);

    // 	printf("A:\n");
    // 	printf("%6.2f %6.2f %6.2f %6.2f %6.2f\n",tM[0][0],tM[0][1],tM[0][2],tM[0][3],tM[0][4]);
    // 	printf("%6.2f %6.2f %6.2f %6.2f %6.2f\n",tM[1][0],tM[1][1],tM[1][2],tM[1][3],tM[1][4]);
    // 	printf("%6.2f %6.2f %6.2f %6.2f %6.2f\n",tM[2][0],tM[2][1],tM[2][2],tM[2][3],tM[2][4]);
    // 	printf("%6.2f %6.2f %6.2f %6.2f %6.2f\n",tM[3][0],tM[3][1],tM[3][2],tM[3][3],tM[3][4]);
    // 	printf("%6.2f %6.2f %6.2f %6.2f %6.2f\n",tM[4][0],tM[4][1],tM[4][2],tM[4][3],tM[4][4]);
}