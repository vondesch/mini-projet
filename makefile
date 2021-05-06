
# Define project name here
PROJECT = mini_projet

#Define path to the e-puck2_main-processor folder
GLOBAL_PATH = ../../lib/e-puck2_main-processor

#Source files to include
CSRC += ./main.c \
		./move.c \
		./wallDetect.c \
		./pid_regulator.c \
		
		

#Header folders to include
INCDIR += 

#Jump to the main Makefile
include $(GLOBAL_PATH)/Makefile