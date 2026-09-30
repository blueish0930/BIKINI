// Copyright (c) 2005-2008 ASCLEPIOS Project, INRIA Sophia-Antipolis (France)
// All rights reserved.
//
// This file is part of the ImageIO Library, and has been adapted for CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.2/CGAL_ImageIO/include/CGAL/ImageIO/fgetns.h $
// $Id: include/CGAL/ImageIO/fgetns.h cac3e9d75e2 $
// SPDX-License-Identifier: LGPL-3.0-or-later
//
//
// Author(s)     :  ASCLEPIOS Project (INRIA Sophia-Antipolis), Laurent Rineau

#ifndef FGETNS_H
#define FGETNS_H

#include <CGAL/ImageIO.h>

namespace CGAL {
/* get a string from a file and discard the ending newline character
   if any */
char *fgetns(char *str, int n,  _image *im );

}
#ifdef CGAL_HEADER_ONLY
#include <CGAL/ImageIO/fgetns_impl.h>
#endif // CGAL_HEADER_ONLY

#endif // FGETNS_H
