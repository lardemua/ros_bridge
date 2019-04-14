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
\brief Callback code for the xgtk plug-in
*/

// System Includes
#include <gtk-3.0/gtk/gtk.h>
#include <graphviz/gvplugin_device.h>
#include <graphviz/gvplugin_render.h>
#include <graphviz/gvplugin.h>
#include <graphviz/gvc.h>
#include <graphviz/gvplugin_layout.h>
#include <graphviz/gvcjob.h>
#include <graphviz/gvcommon.h>
#include <graphviz/gvcext.h>
#include <graphviz/types.h>
#include <graphviz/graph.h>
#include <graphviz/geom.h>
#include <iostream>
//These two includes are needed for the external tree variable
#include <carla_ros_mtt/tree.hh>
#include <carla_ros_mtt/types_declaration.h>
#include <boost/format.hpp>
#include <carla_ros_mtt/graph_wrapper.h>

#include "callbacks.h"//silly prototypes, this include is useless

using namespace std;
using namespace boost;

///External variable linking to the main graph context created in the mht main source code pmht.cpp
extern GVGraph*graph_context;

///External variable linking to the main tree pointer in pmht.cpp
extern tree<t_nodePtr> global_tree;

extern "C" {
extern int agrelabel_node(Agnode_t * n, char *newname);
}

ostream& operator<< (ostream &o, const pointf &i)
{
    return o<<i.x<<" "<<i.y;
}

ostream& operator<< (ostream &o, const boxf &i)
{
    return o<<"LL: "<<i.LL<<" UR: "<<i.UR;
}

gboolean DrawingareaExposeEvent(GtkWidget*widget,GdkEventExpose*event,gpointer user_data)
{
    GVJ_t *job = (GVJ_t *)g_object_get_data(G_OBJECT(widget),"job");
    cairo_t *cr = gdk_cairo_create(widget->window);

    (job->callbacks->motion)(job, job->pointer);

    job->context = (void *)cr;
    job->external_context = true;
    job->width = widget->allocation.width;
    job->height = widget->allocation.height;

// 	job->width =widget->allocation.width*2;
// 	job->height =widget->allocation.height*2;

    tree<t_nodePtr>::iterator parent;

    if(global_tree.ready_from_draw==false || global_tree.need_layout==false)
    {
        (job->callbacks->refresh)(job);
        cairo_destroy(cr);
        return false;
    }

// 	cout<<endl<<endl;
// 	cout<<"Clear all"<<endl;
    graph_context->clearNodes();

// 	cout<<endl<<endl;
// 	cout<<"Create new tree"<<endl;

    for(tree<t_nodePtr>::iterator it = global_tree.begin(); it != global_tree.end(); ++it)
    {
// 		if((*it)->id!=0 && (*it)->id!=1)
// 			continue;

        graph_context->addNode((*it)->GetName());

        switch((*it)->mode)
        {
            case t_node::NEW:
                graph_context->setNodeAttribute((*it)->GetName(),string("color"),string("blue"));
                break;

            case t_node::MAIN:
                graph_context->setNodeAttribute((*it)->GetName(),string("color"),string("black"));
                break;

            case t_node::FAILED:
                graph_context->setNodeAttribute((*it)->GetName(),string("color"),string("red"));
                break;

            default:
                break;
        }

        parent=global_tree.parent(it);

        if(parent!=NULL)
            graph_context->addEdge((*parent)->GetName(),(*it)->GetName());
    }

// 	cout<<endl;
// 	cout<<"Tree created applying layout"<<endl;

    graph_context->applyLayout();
    global_tree.need_layout=false;

    (job->callbacks->refresh)(job);

    cairo_destroy(cr);

    return FALSE;
}

gboolean DrawingareaButtonPressEvent(GtkWidget*widget,GdkEventButton *event,gpointer user_data)
{
    static bool init=true;
    static bool dragging=false;

    if(init)
    {
        g_object_set_data(G_OBJECT(widget),"dragging",&dragging);
        init=false;
    }

    if(event->type==GDK_BUTTON_PRESS)
        dragging=true;
    else if(event->type==GDK_BUTTON_RELEASE)
        dragging=false;

    return true;
}

gboolean DrawingareaMotionNotifyEvent(GtkWidget*widget,GdkEventMotion*event,gpointer user_data)
{
    static bool first_drag=true;

    GVJ_t *job = (GVJ_t *)g_object_get_data(G_OBJECT(widget),"job");

    bool *dragging_p = (bool*)g_object_get_data(G_OBJECT(widget),"dragging");
    if(!dragging_p)
        return FALSE;//the draging variable was not been set yet, it is set by the button press event handler

    bool dragging=*dragging_p;

    static pointf diff;

    double dpix=-1/job->devscale.x;//can also be interpreted as scroling speed
    double dpiy=-1/job->devscale.y;//can also be interpreted as scroling speed

    diff.x=event->x-job->pointer.x;
    diff.y=event->y-job->pointer.y;

    job->pointer.x = event->x;
    job->pointer.y = event->y;

    gtk_widget_queue_draw(widget);

    if(dragging)
    {
        if(first_drag)
        {
            diff.x=0;
            diff.y=0;
            first_drag=false;
        }

        job->focus.x+= diff.x*dpix/job->zoom;
        job->focus.y+= diff.y*dpiy/job->zoom;

        double x_lim = (0.37*job->width-1)/job->zoom;
        double y_lim = (0.37*job->height-1)/job->zoom;

// 		cout<<"Dpi: "<<job->dpi<<endl;
// 		cout<<"View: "<<job->view<<endl;
// 		cout<<"translation: "<<job->translation<<endl;
// 		cout<<"devscale: "<<job->devscale<<endl;
// 		cout<<"focus: "<<job->focus<<endl;
// 		cout<<"scale: "<<job->scale<<endl;

        if(job->focus.x>= x_lim)
            job->focus.x= x_lim;

        if(job->focus.y>= y_lim)
            job->focus.y= y_lim;
    }

    return true;
}

gboolean DrawingareaScrollEvent(GtkWidget*widget,GdkEventScroll *event,gpointer user_data)
{
    if(event->type==GDK_SCROLL)
    {
        GVJ_t *job = (GVJ_t *)g_object_get_data(G_OBJECT(widget),"job");

        pointf pointer;
        pointer.x=event->x;
        pointer.y=event->y;

        if(event->direction==GDK_SCROLL_UP)
            (job->callbacks->button_press)(job,4, pointer);
        else if(event->direction==GDK_SCROLL_DOWN)
            (job->callbacks->button_press)(job,5, pointer);

        double x_lim = (0.37*job->width-1)/job->zoom;
        double y_lim = (0.37*job->height-1)/job->zoom;

        if(job->focus.x>= x_lim)
            job->focus.x= x_lim;

        if(job->focus.y>= y_lim)
            job->focus.y= y_lim;
    }

    return true;
}

gboolean WindowDeleteEvent(GtkWidget*widget,GdkEvent*event,gpointer user_data)
{
    gtk_main_quit();
    return true;
}

gboolean DrawingareaConfigureEvent(GtkWidget*widget,GdkEventConfigure*event,gpointer user_data)
{
    GVJ_t *job;
    double zoom_to_fit;

    /*FIXME - should allow for margins */
    /*      - similar zoom_to_fit code exists in: */
    /*      plugin/gtk/callbacks.c */
    /*      plugin/xlib/gvdevice_xlib.c */
    /*      lib/gvc/gvevent.c */

    job = (GVJ_t *)g_object_get_data(G_OBJECT(widget),"job");

    if (! job->has_been_rendered)
    {
        zoom_to_fit = MIN((double) event->width / (double) job->width,(double) event->height / (double) job->height);
        if (zoom_to_fit < 1.0) /* don't make bigger */
            job->zoom *= zoom_to_fit;
    }else if(job->fit_mode)
    {
        zoom_to_fit = MIN((double) event->width / (double) job->width, (double) event->height / (double) job->height);
        job->zoom *= zoom_to_fit;
    }

    if ((unsigned int)event->width > job->width || (unsigned int)event->height > job->height)
        job->has_grown = TRUE;

    job->width = event->width;
    job->height = event->height;
    job->needs_refresh = TRUE;

    return FALSE;
}