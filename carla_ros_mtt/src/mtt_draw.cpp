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
\brief Draw functions for the old mtt tracking
*/

// System Includes
#include <carla_ros_mtt/mtt_draw.h>

void DrawGlobal(char key,IplImage*img,t_config*config,t_data*data,t_object**objects,int object_size,t_list*list,t_flag*flags,int laser,bool raw_only)
{
    static bool d_help=true;
    static bool d_occlusionzone=false;
    static bool d_velocity=true;
    static bool d_objects=true;
    static bool d_raw=false;
    static bool d_searcharea=true;
    static bool d_paths=true;
    static bool d_ids=true;
    char help_msg[1024];
    static bool init=true;

    static CvFont fontTitle;
    static CvFont fontText;

    if(init)
    {
        if(raw_only)
        {
            d_help=false;
            d_occlusionzone=false;
            d_velocity=false;
            d_objects=false;
            d_raw=true;
            d_searcharea=false;
            d_paths=false;
            d_ids=false;
        }
        cvInitFont(&fontTitle,CV_FONT_HERSHEY_SIMPLEX ,.6,.6,0.,1,CV_AA);
        cvInitFont(&fontText,CV_FONT_HERSHEY_SIMPLEX,.4,.4,0.,1,CV_AA);
        init=false;
    }

    switch(key)
    {
        case 'h':
        case 'H':
            d_help=!d_help;
            break;
        case 'z':
        case 'Z':
            d_occlusionzone=!d_occlusionzone;
            break;
        case 'v':
        case 'V':
            d_velocity=!d_velocity;
            break;
        case 'o':
        case 'O':
            d_objects=!d_objects;
            break;
        case 'r':
        case 'R':
            d_raw=!d_raw;
            break;
        case 's':
        case 'S':
            d_searcharea=!d_searcharea;
            break;
        case 'p':
        case 'P':
            d_paths=!d_paths;
            break;
        case 'i':
        case 'I':
            d_ids=!d_ids;
            break;
    }

    draw_ambient(img,config);		//draw ambient on output image

    if(d_occlusionzone)
        draw_oclusion_area(img,objects,object_size,CV_RGB(50,50,155),config);

    if(d_raw)
        draw_raw_data(img,data,CV_RGB(0,255,0),config,flags);

    if(d_objects)
    {
        draw_objects(img,objects,object_size,CV_RGB(255,0,0),config,flags);
        draw_objects_centers(img,objects,object_size,CV_RGB(0,255,0),config,flags);
    }

    if(d_paths)
    {
        DrawListPaths(img,list,config);
        DrawListCenters(img,list,CV_RGB(255,255,255),config);
    }

    if(d_ids)
        DrawListIds(img,list,CV_RGB(0,200,0),config);

    if(d_searcharea)
        DrawSearchArea(img,list,config);

    if(d_velocity)
        DrawVelocity(img,list,config);

    if(d_help)
    {
        //Draw help message

        strcpy(help_msg,"Keyboard Shortcuts");
        cvPutText(img,help_msg,cvPoint(10,20),&fontTitle,CV_RGB(255,120,71));
        strcpy(help_msg,"h - this message");
        cvPutText(img,help_msg,cvPoint(10,40),&fontText,CV_RGB(200,200,71));
        strcpy(help_msg,"z - occlusion zones");
        cvPutText(img,help_msg,cvPoint(10,60),&fontText,CV_RGB(200,200,71));
        strcpy(help_msg,"v - velocity");
        cvPutText(img,help_msg,cvPoint(10,80),&fontText,CV_RGB(200,200,71));
        strcpy(help_msg,"o - objects");
        cvPutText(img,help_msg,cvPoint(10,100),&fontText,CV_RGB(200,200,71));
        strcpy(help_msg,"r - raw data");
        cvPutText(img,help_msg,cvPoint(10,120),&fontText,CV_RGB(200,200,71));
        strcpy(help_msg,"s - search zones");
        cvPutText(img,help_msg,cvPoint(10,140),&fontText,CV_RGB(200,200,71));
        strcpy(help_msg,"p - paths");
        cvPutText(img,help_msg,cvPoint(10,160),&fontText,CV_RGB(200,200,71));
        strcpy(help_msg,"i - ids");
        cvPutText(img,help_msg,cvPoint(10,180),&fontText,CV_RGB(200,200,71));
        strcpy(help_msg,"q - quit");
        cvPutText(img,help_msg,cvPoint(10,200),&fontText,CV_RGB(200,200,71));
    }
}

void DrawListPaths(IplImage*img,t_list*list,t_config*config)
{
    CvPoint A;
    CvPoint B;

    while(list!=null)
    {
        if(list->timers.lifetime<config->display_min_lifetime)
        {
            list=list->next;
            continue;
        }

        if(list->classification.velocity_classification==STATIONARY)
        {
            list=list->next;
            continue;
        }

        if(list->model==CV)
        {
            for(unsigned int i=0;i<list->path_cv.number_points-1;i++)
            {
                if(i==list->path_cv.position-1)
                    continue;
                //
                A=cvPoint(real2print(list->path_cv.x[i],config),real2print(list->path_cv.y[i],config));
                B=cvPoint(real2print(list->path_cv.x[i+1],config),real2print(list->path_cv.y[i+1],config));

                if(A.x > img->width || A.y > img->height || A.x < 0|| A.y < 0 || B.x > img->width || B.y > img->height || B.x < 0|| B.y < 0)
                {
                    //line out side image
                }else
                    cvLine(img,A,B,CV_RGB(220,150,50),1,8,0);
            }
        }
        else
        {
            for(unsigned int i=0;i<list->path_ca.number_points-1;i++)
            {
                if(i==list->path_ca.position)
                    continue;

                A=cvPoint(real2print(list->path_ca.x[i],config),real2print(list->path_ca.y[i],config));
                B=cvPoint(real2print(list->path_ca.x[i+1],config),real2print(list->path_ca.y[i+1],config));

                if(A.x > img->width || A.y > img->height || A.x < 0|| A.y < 0 || B.x > img->width || B.y > img->height || B.x < 0|| B.y < 0)
                {
                    //line out side image
                }else
                    cvLine(img,A,B,CV_RGB(220,0,150),1,8,0);
            }
        }
        list=list->next;
    }
}

void DrawSearchArea(IplImage*img,t_list*list,t_config*config)
{
    CvPoint I;
    double px,py,ellipse_A,ellipse_B,ellipse_angle;

    while(list!=null)
    {
        if(list->timers.lifetime<config->display_min_lifetime)
        {
            list=list->next;
            continue;
        }

        if(list->classification.velocity_classification==STATIONARY)
        {
            list=list->next;
            continue;
        }

        px=list->position.predicted_x;
        py=list->position.predicted_y;

        I=cvPoint(real2print(px,config),real2print(py,config));

        ellipse_A=list->search_area.ellipse_A;
        ellipse_B=list->search_area.ellipse_B;
        ellipse_angle=list->search_area.angle;

        if(list->timers.occludedtime > 0){}
            // 			cvEllipse(img,I, cvSize(real2print(ellipse_A,config),real2print(ellipse_B,config)),ellipse_angle*180./M_PI,360,0,CV_RGB(150,0,150),1,8,0);
        else
            cvEllipse(img,I, cvSize(real2print(ellipse_A,config),real2print(ellipse_B,config)),ellipse_angle*180./M_PI,360,0,CV_RGB(50,250,100),1,8,0);

        list=list->next;
    }
}

void DrawListCenters(IplImage*img,t_list*list,CvScalar color,t_config*config)
{
    double x,y;
    CvPoint a;

    while(list!=null)
    {
        if(list->timers.lifetime<config->display_min_lifetime)
        {
            list=list->next;
            continue;
        }

        x=list->position.estimated_x;
        y=list->position.estimated_y;

        a=cvPoint(real2print(x,config),real2print(y,config));

        cvLine(img,a,a,color,4, 8, 0 );

        list=list->next;
    }
}

void DrawVelocity(IplImage*img,t_list*list,t_config*config)
{
    double x,y;
    CvPoint a;

    while(list!=null)
    {
        if(list->timers.lifetime<config->display_min_lifetime)
        {
            list=list->next;
            continue;
        }

        if(list->classification.velocity_classification==STATIONARY)
        {
            list=list->next;
            continue;
        }

        x=list->position.estimated_x;
        y=list->position.estimated_y;

        a=cvPoint(real2print(x,config),real2print(y,config));
        double velocity_angle=atan2(list->velocity.velocity_y,list->velocity.velocity_x);
        drawarrow(a.x,a.y,velocity_angle, real2print(list->velocity.velocity_module,config), img, CV_RGB(0,100,255),2, 8, 0);

        list=list->next;
    }
}

void DrawListIds(IplImage*img,t_list*list,CvScalar color,t_config*config)
{
    static CvFont font;
    double x,y;
    static bool initialise=true;
    char text[100];

    if(initialise)
    {
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,.5,.5,0.,1,8);
        initialise=false;
    }

    while(list!=null)
    {
        if(list->timers.lifetime<config->display_min_lifetime)
        {
            list=list->next;
            continue;
        }

        x=list->position.estimated_x;
        y=list->position.estimated_y;

        // 		sprintf(text,"%d %d",list->id,list->timers.occludedtime);
        // 		sprintf(text,"%d %c",list->id,list->model==CV?'V':'A');
        sprintf(text,"%d",list->id);
        if(list->classification.velocity_classification==STATIONARY)
            cvPutText(img,text, cvPoint(real2print(x,config),real2print(y,config)),&font,color);
        else
            cvPutText(img,text, cvPoint(real2print(x,config),real2print(y,config)),&font,CV_RGB(255,0,0));

        list=list->next;
    }
}

/**
* @brief Draws a point in objects centers
* @param img Image to draw on
* @param objects Objects to be drawn
* @param size number of objects
* @param color color to be used
* @param config general config structure
* @return void
*/
void draw_objects_centers(IplImage*img,t_object**objects,int size,CvScalar color,t_config*config)
{
    CvPoint a;
    int i;

    double x,y;

    for(i=0;i<size;i++)
    {
        x=objects[i]->cx;
        y=objects[i]->cy;

        a=cvPoint(real2print(x,config),real2print(y,config));

        if(objects[i]->object_found==true)
            cvLine(img,a,a,color,4, 8, 0 );
        else
            cvLine(img,a,a,CV_RGB(253,0,0),4, 8, 0 );
    }
}

/**
* @brief Draw objects ID's in their centers
* @param img Image to draw on
* @param objects Objects to be drawn
* @param size number of objects
* @param color color to be used
* @param config general config structure
* @return void
*/
void draw_objects_ids(IplImage*img,t_object**objects,int size,CvScalar color,t_config*config)
{
    static CvFont font;
    double x,y;
    static bool initialise=true;
    char text[100];
    int i;

    if(initialise)
    {
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,.5,.5,0.,1,8);
        initialise=false;
    }

    for(i=0;i<size;i++)
    {
        x=objects[i]->cx;
        y=objects[i]->cy;

        sprintf(text,"%d",objects[i]->id);
        cvPutText(img,text, cvPoint(real2print(x,config),real2print(y,config)),&font,color);
    }
}

/**
* @brief Draw objects lines
*
* @param img Image to draw on
* @param objects Objects to be drawn
* @param size number of on_linesbjects
* @param color color to be used
* @param config general config structure
* @return void
*/
void draw_objects(IplImage*img,t_object**objects,int size,CvScalar color,t_config*config)
{
    CvPoint a,b;
    int i,e;

    for(i=0;i<size;i++)
    {
        for(e=0;e<objects[i]->n_lines;e++)
        {
            a=cvPoint(real2print(objects[i]->line[e]->xi,config),real2print(objects[i]->line[e]->yi,config));
            b=cvPoint(real2print(objects[i]->line[e]->xf,config),real2print(objects[i]->line[e]->yf,config));

            if(objects[i]->object_found==true)
                cvLine(img,a,b,color,2, 8, 0 );
            else
                cvLine(img,a,b,CV_RGB(253,156,64),2, 8, 0 );
        }
    }
}

/**
* @brief Draw clusters end lines, lines from the center of the image to the first and last point of the cluster
*
* @param img Image to draw on
* @param clusters clusters to be drawn
* @param size number of clusters
* @param data general data storage vector
* @param config general config structure
* @return void
*/
void draw_end_lines(IplImage*img,t_cluster**clusters,int size,t_data*data,t_config*config)
{
    CvPoint O,I;
    int i;

    O=cvPoint(config->w/2,config->h/2);

    for(i=0;i<size;i++)
    {
        I=cvPoint(real2print(data->x[clusters[i]->stp],config),real2print(data->y[clusters[i]->stp],config));
        cvLine(img,O,I,CV_RGB(255,0,0),1, 8, 0 );

        I=cvPoint(real2print(data->x[clusters[i]->enp-1],config),real2print(data->y[clusters[i]->enp-1],config));
        cvLine(img,O,I,CV_RGB(0,0,255),1, 8, 0 );
    }
}

/**
* @brief Prints the cluster number of points
*
* @param img Image to draw on
* @param clusters clusters to be drawn
* @param size number of clusters
* @param data general data storage vector
* @param config general config structure
* @return void
*/
void draw_clusters_npoints(IplImage*img,t_cluster**clusters,int size,t_data*data,t_config*config)
{
    static char text[512];
    static CvFont font;
    static int initialise=true;

    if(initialise)
    {
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,.5,.5,0.,1,8);
        initialise=false;
    }

    CvPoint I;
    int i;

    for(i=0;i<size;i++)
    {
        I=cvPoint(real2print(data->x[clusters[i]->stp],config),real2print(data->y[clusters[i]->stp],config));

        sprintf(text,"NP %d",clusters[i]->n_points);
        cvPutText(img,text,I,&font,CV_RGB(50,255,0));
    }
}

/**
* @brief Draw background ambient
*
* @param img Image to draw on
* @param config general config structure
* @param style style of the background ambient
* @param laser id of the laser
* @return void
*/
void draw_ambient(IplImage*img,t_config*config,enum_background_style style,int laser)
{

    static CvPoint curve[4];
    static CvScalar ColorLaser,ColorBackground,ColorForeground;

    static bool init=true;

    if(init)
    {
        if(laser==2)
        {
            // 			curve[0]=cvPoint(config->w/2,config->h);
            // 			curve[1]=cvPoint(config->w/2,config->h/2);
            // 			curve[2]=cvPoint(config->w,config->h/2);
            // 			curve[3]=cvPoint(config->w,config->h);
            curve[0]=cvPoint(0,config->h);
            curve[1]=cvPoint(config->w/2,config->h/2);
            curve[2]=cvPoint(config->w,config->h);
            curve[3]=cvPoint(config->w,config->h);
        }else if(laser==3)
        {
            curve[0]=cvPoint(0,config->h/2);
            curve[1]=cvPoint(config->w/2,config->h/2);
            curve[2]=cvPoint(config->w/2,config->h);
            curve[3]=cvPoint(0,config->h);
        }else
        {
            curve[0]=cvPoint(0,config->h);
            curve[1]=cvPoint(config->w/2,config->h/2);
            curve[2]=cvPoint(config->w,config->h);
            curve[3]=cvPoint(config->w,config->h);
        }

        switch(style)
        {
            case STYLE_DARK:
                ColorLaser=CV_RGB(50,255,0);
                ColorBackground=CV_RGB(0,0,0);
                ColorForeground=CV_RGB(50,50,50);
                break;

            case STYLE_LIGHT:
                ColorLaser=CV_RGB(50,255,0);
                ColorBackground=CV_RGB(255,255,255);
                ColorForeground=CV_RGB(100,100,100);
                break;

            default:
                ColorLaser=CV_RGB(50,255,0);
                ColorBackground=CV_RGB(0,0,0);
                ColorForeground=CV_RGB(50,50,50);
                break;
        }

        init=false;
    }



    cvSet( img, ColorForeground,NULL );	//clear image
    cvCircle(img, cvPoint(config->w/2,config->h/2),real2print(config->out_clip,config), ColorBackground,CV_FILLED );	//outter cliping
    cvCircle(img, cvPoint(config->w/2,config->h/2),real2print(config->in_clip,config), ColorForeground,CV_FILLED );	//inner cliping
    cvFillConvexPoly(img,curve,4,ColorForeground,8,0);

    // 	cvCircle(img, cvPoint(config->w/2,config->h/2), 2, ColorLaser, CV_FILLED );	//laser position
}

/**
* @brief Draw a circle on the midle of the image
*
* @param img Image to draw on
* @param config general config structure
* @return void
*/
void draw_midle_circle(IplImage*img,t_config*config)
{
    cvCircle(img, cvPoint(config->w/2,config->h/2),real2print(config->in_clip,config), CV_RGB(50,50,50),CV_FILLED );	//inner cliping
}

/**
* @brief Draw a point on the cluster in r minimum and mean theta
*
* @param img Image to draw on
* @param clusters clusters to be drawn
* @param size number of clusters
* @param config general config structure
* @return void
*/
void draw_clusters_centers(IplImage*img,t_cluster**clusters,int size,t_config*config)
{
    int i;
    CvPoint a;
    double x,y,r,t;

    for(i=0;i<size;i++)
    {
        r=clusters[i]->rmin;
        t=clusters[i]->tm;

        x=config->maxr+r*cos(t);
        y=config->maxr-r*sin(t);

        a=cvPoint(real2print(x,config),real2print(y,config));
        cvLine(img,a,a,CV_RGB(0,255,0),6,8,0);
    }
}


/**
* @brief Draw the cluster inner area
*
* @param img Image to draw on
* @param size number of objects
* @param list list of objects
* @param color color to draw
* @param config general config structure
* @return void
*/
void draw_oclusion_area(IplImage*img,t_object**list,int size,CvScalar color,t_config*config)
{
    CvPoint O=cvPoint(config->w/2,config->h/2),line[4];
    double theta_i,theta_f;

    for(int i=0;i<size;i++)
    {
        for(int e=0;e<list[i]->n_lines;e++)
        {
            line[0].x=real2print(list[i]->line[e]->xi,config);
            line[0].y=real2print(list[i]->line[e]->yi,config);
            theta_i=atan2(-(line[0].y-O.y),(line[0].x-O.x));

            line[1].x=real2print(list[i]->line[e]->xf,config);
            line[1].y=real2print(list[i]->line[e]->yf,config);
            theta_f=atan2(-(line[1].y-O.y),(line[1].x-O.x));

            line[3].x=config->maxr+1*config->maxr*cos(theta_i);	//real value in (mm)
            line[3].y=config->maxr-1*config->maxr*sin(theta_i);	//real value in (mm)

            line[2].x=config->maxr+1*config->maxr*cos(theta_f);	//real value in (mm)
            line[2].y=config->maxr-1*config->maxr*sin(theta_f);	//real value in (mm)

            for(int f=2;f<4;f++)
            {
                line[f].x=real2print(line[f].x,config);
                line[f].y=real2print(line[f].y,config);
            }

            cvFillConvexPoly(img,line,4,color,8,0);
        }
    }
}

/**
* @brief Draw the cluster inner area
*
* @param img Image to draw on
* @param clusters clusters to be drawn
* @param size number of clusters
* @param config general config structure
* @param data general data storage vector
* @return void
*/
void draw_clusters_area(IplImage*img,t_cluster**clusters,int size,t_config*config,t_data*data)
{
    int i,e;
    CvPoint a,b;
    static CvPoint O;

    CvPoint line[]={a,b,O};

    static bool initialise=true;

    if(initialise)
    {
        O=cvPoint(config->w/2,config->h/2);
        initialise=false;
    }

    for(i=0;i<size;i++)
        for(e=clusters[i]->stp;e<clusters[i]->enp;e++)
        {
            a=cvPoint(real2print(data->x[e],config),real2print(data->y[e],config));
            b=cvPoint(real2print(data->x[e+1],config),real2print(data->y[e+1],config));

            line[0]=O,line[1]=a;line[2]=b;
            cvFillConvexPoly(img,line,3,CV_RGB(232,197,76),8,0);
        }
}

/**
* @brief Draw all cluster points connect by lines
*
* @param img Image to draw on
* @param clusters clusters to be drawn
* @param size number of clusters
* @param data general data storage vector
* @param config general config structure
* @return void
*/
void draw_clusters(IplImage*img,t_cluster**clusters,int size,t_data*data,t_config*config)
{
    int i,e;
    CvPoint a;
    double R,G,B;

    R=255;
    G=255;
    B=255;

    for(e=0;e<size;e++)	//number of clusters
    {/*
	R=rand()%150+100;
	G=rand()%100+150;
	B=rand()%256;*/

        for(i=clusters[e]->stp;i<=clusters[e]->enp;i++)
        {
            a=cvPoint(real2print(data->x[i],config),real2print(data->y[i],config));
            cvLine(img,a,a,CV_RGB(R,G,B),2, 8, 0 );
        }

        // 		cvShowImage("Laser range image",img);
        // 		cvWaitKey(0);
    }
}

/**
* @brief Draw raw data
*
* @param img Image to draw on
* @param data general data storage vector
* @param color color to be used
* @param config general config structure
* @return void
*/
void draw_raw_data(IplImage*img,t_data*data,CvScalar color,t_config*config)
{
    int i;
    CvPoint a;
    for(i=0;i<data->n_points;i++)
    {
        a=cvPoint(real2print(data->x[i],config),real2print(data->y[i],config));
        if(data->flag[i]==false)
        {
            // 			if(data->occlusion_data==false)
            cvLine(img,a,a,color,2,8,0);
        }else
            cvLine(img,a,a,CV_RGB(255,220,0),2,8,0);

        // 	cvShowImage("Laser range image",img);
        // 	cvWaitKey(2);
    }

}

/**
* @brief Draw raw data accumulator
*
* @param img Image to draw on
* @param data_acc general data storage accumulator vector
* @param color color to be used
* @param config general config structure
* @return void
*/
void draw_raw_data_acc(IplImage*img,t_data_acc*data_acc,CvScalar color,t_config*config)
{
    int i,e;
    CvPoint a;
    for(i=0;i<data_acc->n_scans;i++)
        for(e=0;e<data_acc->data[i]->n_points;e++)
        {
            a=cvPoint(real2print(data_acc->data[i]->x[e],config),real2print(data_acc->data[i]->y[e],config));
            cvLine(img,a,a,color,2,8,0);
        }
}

/**
* @brief Draw arrow
*
* @param x0 start x position
* @param y0 start y position
* @param o orientatio angle
* @param lenght length of the arrow
* @param dst destination image
* @param color color of the arrow
* @param thickness thickness of the line
* @param line_type line type
* @param shift i don't know what this is
* @return void
*/
void drawarrow(int x0, int y0, double o, double lenght, IplImage *dst, CvScalar color, int thickness, int line_type, int shift)
{
    int x1,y1;
    x1 = x0 + lenght*cos(o);
    y1 = y0 + lenght*sin(o);

    CvPoint pt1 = cvPoint(x1 - lenght/3*cos(o+M_PI/4), y1 - lenght/3*sin(o+M_PI/4));
    CvPoint pt2 = cvPoint(x1 - lenght/3*cos(o-M_PI/4), y1 - lenght/3*sin(o-M_PI/4));

    cvLine( dst, cvPoint(x0,y0),cvPoint(x1,y1), color,thickness, line_type, shift);

    cvLine( dst, cvPoint(x1,y1),pt1, color,thickness, line_type, shift);
    cvLine( dst, cvPoint(x1,y1),pt2, color,thickness, line_type, shift);
}