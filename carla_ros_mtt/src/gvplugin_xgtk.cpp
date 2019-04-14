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
\brief Main xgtk plug-in implementation, global vars, engines and apis including the all important gvplugin_xgtk_LTX_library
*/

// System Includes
#include <gtk-3.0/gtk/gtk.h>
#include <graphviz/gvc.h>
#include <graphviz/gvplugin.h>
#include <graphviz/gvplugin_device.h>
#include <graphviz/types.h>
#include <inttypes.h>
#include <cairo/cairo.h>
#include <iostream>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <signal.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>
#include <string.h>
#include "callbacks.h"

/**
\brief Hookup object to widget with call to g_object_set_data_full
Link a widget to an other widget by a specific name.\n
The linked widget can latter be retrieved using \c g_object_get_data,
the current macro is in fact just a wrapper to the \c g_object_set_data_full GTK function.
*/
#define GLADE_HOOKUP_OBJECT(component,widget,name) \
  g_object_set_data_full (G_OBJECT (component), name, \
    gtk_widget_ref (widget), (GDestroyNotify) gtk_widget_unref)

/**
\brief Hookup object to widget with call to g_object_set_data
Simpler version of the GLADE_HOOKUP_OBJECT that uses \c g_object_set_data instead of
\c g_object_set_data_full.
*/
#define GLADE_HOOKUP_OBJECT_NO_REF(component,widget,name) \
  g_object_set_data (G_OBJECT (component), name, widget)

/**
\brief Obtain a widget from a lower point in the widget structure
This function works it way up the parent of the widget until it reaches the desired widget.\n
This function is used in conjunction with macros GLADE_HOOKUP_OBJECT and GLADE_HOOKUP_OBJECT_NO_REF.\n
It will not work if the widget was not added using these macros.
\param widget the source
\param widget_name reference name of the widget to obtain
\return pointer to the desired widget
*/
GtkWidget*lookup_widget(GtkWidget*widget,const gchar*widget_name)
{
    GtkWidget *parent, *found_widget;

    for (;;)
    {
        if (GTK_IS_MENU (widget))
            parent = gtk_menu_get_attach_widget (GTK_MENU (widget));
        else
            parent = widget->parent;

        if (!parent)
            parent = (GtkWidget*) g_object_get_data (G_OBJECT (widget), "GladeParentKey");

        if (parent == NULL)
            break;

        widget = parent;
    }

    found_widget = (GtkWidget*) g_object_get_data (G_OBJECT (widget),widget_name);

    if (!found_widget)
        g_warning ("Widget not found: %s", widget_name);

    return found_widget;
}

/**
\brief Create a GTK window
This function creates a window that will contain the representation of the graph.
\return pointer to the top level window.
*/
GtkWidget*create_window(void)
{
    GtkWidget *window;
    GtkWidget *drawingarea;

    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title (GTK_WINDOW (window), "Tree Graph");

    drawingarea = gtk_drawing_area_new ();
    gtk_widget_show (drawingarea);
    gtk_widget_set_size_request (drawingarea, 800, 800);
    gtk_container_add (GTK_CONTAINER (window), drawingarea);

    g_signal_connect ((gpointer) window, "delete_event",G_CALLBACK (WindowDeleteEvent),NULL);
    g_signal_connect ((gpointer) drawingarea, "expose_event",G_CALLBACK (DrawingareaExposeEvent),NULL);
    g_signal_connect ((gpointer) drawingarea, "motion_notify_event",G_CALLBACK (DrawingareaMotionNotifyEvent),NULL);
    g_signal_connect ((gpointer) drawingarea, "configure_event",G_CALLBACK (DrawingareaConfigureEvent),NULL);
    g_signal_connect ((gpointer) drawingarea, "button-press-event",G_CALLBACK (DrawingareaButtonPressEvent),NULL);
    g_signal_connect ((gpointer) drawingarea, "button-release-event",G_CALLBACK (DrawingareaButtonPressEvent),NULL);
    g_signal_connect ((gpointer) drawingarea, "scroll-event",G_CALLBACK (DrawingareaScrollEvent),NULL);

    gtk_widget_set_events (drawingarea, GDK_ENTER_NOTIFY_MASK |GDK_EXPOSURE_MASK | GDK_LEAVE_NOTIFY_MASK | GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK | GDK_POINTER_MOTION_HINT_MASK);

    /* Store pointers to all widgets, for use by lookup_widget(). */
    GLADE_HOOKUP_OBJECT_NO_REF (window, window, "window");
    GLADE_HOOKUP_OBJECT (window, drawingarea, "drawingarea");

    return window;
}

/**
\brief Redraw main window
This function invalidates the main window which makes it redraw itself, this is a way to periodically call the expose event.
\param data input data pointer, the window widget to invalidate is received here
\return always true to keep the redrawing always alive
*/
gboolean Redraw(gpointer data)
{
    GtkWidget*window=(GtkWidget*)data;
    GtkWidget*drawingarea = lookup_widget (window, "drawingarea");

    if(drawGraph())
        gdk_window_invalidate_rect(drawingarea->window,&drawingarea->allocation,FALSE);
    return 1;
}

/**
\brief Start point for the plug-in
This function is the start point of the \c gvdevice_engine_t object.\n
Making it the start point of the render plug-in.
It is responsible for starting GTK and initializing the display.\n
It can <b>NOT</b> start the \c gtk_main function.
\param firstjob job structure of the plug-in
*/
static void gtk_initialize(GVJ_t *firstjob)
{
    Display *dpy;
    const char *display_name = NULL;
    int scr;

    gtk_set_locale ();
    gtk_init (NULL, NULL);

    dpy = XOpenDisplay(display_name);
    if (dpy == NULL) {
        fprintf(stderr, "Failed to open XLIB display: %s\n",
                XDisplayName(NULL));
        return;
    }
    scr = DefaultScreen(dpy);

    firstjob->device_dpi.x = DisplayWidth(dpy, scr) * 25.4 / DisplayWidthMM(dpy, scr);
    firstjob->device_dpi.y = DisplayHeight(dpy, scr) * 25.4 / DisplayHeightMM(dpy, scr);

    firstjob->device_sets_dpi = TRUE;
}

/**
\brief End point for the plug-in engine
This function is the end point for the \c gvdevice_engine_t object.\n
This function shows the main window and starts the \c gtk_main object.
\param firstjob job structure of the plug-in, and only by the way
*/
static void gtk_finalize(GVJ_t *firstjob)
{
    GVJ_t *job;
    GtkWidget *window, *drawingarea;

    for (job = firstjob; job; job = job->next_active)
    {
        window = create_window();

        g_object_set_data(G_OBJECT(window), "job", (gpointer) job);

        drawingarea = lookup_widget (window, "drawingarea");
        g_object_set_data(G_OBJECT(drawingarea), "job", (gpointer) job);

        g_timeout_add(50,Redraw,window);

        gtk_widget_show (window);
    }

    gtk_main();
}

///Libgvc plug-in render engine description
static gvdevice_engine_t device_engine_xgtk = {
        gtk_initialize,
        NULL,			/* gtk_format */
        gtk_finalize,
};

///Libgvc plug-in render feature descriptor
static gvdevice_features_t device_features_xgtk = {
        GVDEVICE_DOES_TRUECOLOR	| GVDEVICE_EVENTS ,      /* flags */
        {0.,0.},                    /* default margin - points */
        {100.,100.},                    /* default page width, height - points */
        {96.,96.},                  /* dpi */
};

///Libgvc plug-in descriptor, name. engine and features, must be null terminated
gvplugin_installed_t gvdevice_types_xgtk[] = {
        {0,(char*)"xgtk:cairo", 0, &device_engine_xgtk, &device_features_xgtk},
        {0, NULL, 0, NULL, NULL}
};

///Libgvc api descriptor, type and \c gvplugin_installed_t structure
static gvplugin_api_t apis[] = {
        {API_device, gvdevice_types_xgtk},
        {(api_t)0, 0},
};

///Libgvc plug-in library object, the name of this variable is very specific (its specification is in the Libgvc documentation)
gvplugin_library_t gvplugin_xgtk_LTX_library = {(char*)"xgtk", apis };