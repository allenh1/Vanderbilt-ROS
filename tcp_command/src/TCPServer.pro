TEMPLATE   = app
CONFIG	  += qt warn_on no_keywords console embed_manifest_exe
QT        += network
QT        -= gui
TARGET     = TCPServer
HEADERS	   = Server.h RobotThread.h
SOURCES	   = main.cpp Server.cpp RobotThread.cpp
LIBS      += 

unix:CONFIG += link_pkgconfig
unix:PKGCONFIG += playerc++

# Treat warnings as errors
win32:QMAKE_CXXFLAGS += /WX

CONFIG(debug, debug|release){
	# Debug build options
}
else{
	# Release build options
}
