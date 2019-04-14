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

#ifndef ROS_BRIDGE_CALLBACKS_H
#define ROS_BRIDGE_CALLBACKS_H

/**
\file
\brief Callback header, only function prototypes
*/

// System Includes
#include <gtk-3.0/gtk/gtk.h>

gboolean DrawingareaExposeEvent(GtkWidget*widget,GdkEventExpose*event,gpointer user_data);
gboolean DrawingareaMotionNotifyEvent(GtkWidget*widget,GdkEventMotion*event,gpointer user_data);
gboolean DrawingareaExposeEvent(GtkWidget*widget,GdkEventExpose*event,gpointer user_data);
gboolean WindowDeleteEvent(GtkWidget*widget,GdkEvent*event,gpointer user_data);
gboolean DrawingareaConfigureEvent(GtkWidget*widget,GdkEventConfigure*event,gpointer user_data);
gboolean DrawingareaButtonPressEvent(GtkWidget*widget,GdkEventButton *event,gpointer user_data);
gboolean DrawingareaScrollEvent(GtkWidget*widget,GdkEventScroll *event,gpointer user_data);
gboolean drawGraph(void);

#endif //ROS_BRIDGE_CALLBACKS_H
