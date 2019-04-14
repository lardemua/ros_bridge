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

#ifndef ROS_BRIDGE_TREE_UTIL_H
#define ROS_BRIDGE_TREE_UTIL_H


/**
\file
\brief Utility functions for the tree class
A collection of miscellaneous utilities that operate on the templated
tree.hh class.
Copyright (C) 2001-2009  Kasper Peeters <kasper.peeters@aei.mpg.de>
(At the moment this only contains a printing utility, thanks to Linda
Buisman <linda.buisman@studentmail.newcastle.edu.au>)
This program is free software: you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// System Includes
#include <iostream>
#include "tree.h"

namespace kptree {

    template<class T>
    void print_tree_bracketed(const tree<T>& t, std::ostream& str=std::cout);

    template<class T>
    void print_subtree_bracketed(const tree<T>& t, typename tree<T>::iterator iRoot,
                                 std::ostream& str=std::cout);

// Iterate over all roots (the head) and print each one on a new line
// by calling printSingleRoot.

    template<class T>
    void print_tree_bracketed(const tree<T>& t, std::ostream& str)
    {
        std::cout<<std::endl;
        int headCount = t.number_of_siblings(t.begin());
        int headNum = 0;
        for(typename tree<T>::sibling_iterator iRoots = t.begin(); iRoots != t.end(); ++iRoots, ++headNum) {
            print_subtree_bracketed(t,iRoots,str);
            if (headNum != headCount) {
                str << std::endl;
            }
        }

        std::cout<<std::endl;
    }


// Print everything under this root in a flat, bracketed structure.

    template<class T>
    void print_subtree_bracketed(const tree<T>& t, typename tree<T>::iterator iRoot, std::ostream& str)
    {
        if(t.empty()) return;
        if (t.number_of_children(iRoot) == 0) {
            str << *iRoot;
        }
        else {
            // parent
            str << *iRoot;
            str << "(";
            // child1, ..., childn
            int siblingCount = t.number_of_siblings(t.begin(iRoot));
            int siblingNum;
            typename tree<T>::sibling_iterator iChildren;
            for (iChildren = t.begin(iRoot), siblingNum = 0; iChildren != t.end(iRoot); ++iChildren, ++siblingNum) {
                // recursively print child
                print_subtree_bracketed(t,iChildren,str);
                // comma after every child except the last one
                if (siblingNum != siblingCount ) {
                    str << ", ";
                }
            }
            str << ")";
        }
    }
}
#endif //ROS_BRIDGE_TREE_UTIL_H
