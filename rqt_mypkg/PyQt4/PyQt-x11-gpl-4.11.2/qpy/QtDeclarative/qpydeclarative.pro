# This is the qmake project file for the QPy support code for the QtDeclarative
# module.  Note that it is not required by configure-ng.py.
#
# Copyright (c) 2014 Riverbank Computing Limited <info@riverbankcomputing.com>
# 
# This file is part of PyQt.
# 
# This file may be used under the terms of the GNU General Public
# License versions 2.0 or 3.0 as published by the Free Software
# Foundation and appearing in the files LICENSE.GPL2 and LICENSE.GPL3
# included in the packaging of this file.  Alternatively you may (at
# your option) use any later version of the GNU General Public
# License if such license has been publicly approved by Riverbank
# Computing Limited (or its successors, if any) and the KDE Free Qt
# Foundation. In addition, as a special exception, Riverbank gives you
# certain additional rights. These rights are described in the Riverbank
# GPL Exception version 1.1, which can be found in the file
# GPL_EXCEPTION.txt in this package.
# 
# If you are unsure which license is appropriate for your use, please
# contact the sales department at sales@riverbankcomputing.com.
# 
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.


QT          += declarative
CONFIG      += static warn_on
TARGET      = qpydeclarative
TEMPLATE    = lib

SOURCES   = \
            qpydeclarative_chimera_helpers.cpp \
            qpydeclarative_listwrapper.cpp \
            qpydeclarative_post_init.cpp \
            qpydeclarativelistproperty.cpp

HEADERS   = \
            qpydeclarative_api.h \
            qpydeclarative_chimera_helpers.h \
            qpydeclarative_listwrapper.h \
            qpydeclarativelistproperty.h \
            qpydeclarativepropertyvaluesource.h
