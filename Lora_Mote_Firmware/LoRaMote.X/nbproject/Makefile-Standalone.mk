#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-Standalone.mk)" "nbproject/Makefile-local-Standalone.mk"
include nbproject/Makefile-local-Standalone.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=Standalone
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/LoRaMote.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/LoRaMote.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=main.c ../Includes/Board/poweruptest.c ../Includes/Board/MOTEapp.c ../Includes/Board/USBapp.c ../Includes/Drivers/buttons.c ../Includes/Drivers/TMRapp.c ../Includes/Drivers/SSD1306oLED.c ../Includes/MccGenerated/eusart.c ../Includes/MccGenerated/interrupt_manager.c ../Includes/MccGenerated/mcc.c ../Includes/MccGenerated/adc.c ../Includes/MccGenerated/spi.c ../Includes/MccGenerated/tmr1.c ../Includes/MccGenerated/pin_manager.c ../Includes/MccGenerated/memory.c ../Includes/MccGenerated/cmp1.c ../Includes/MccGenerated/fvr.c ../Includes/MccGenerated/tmr2.c ../Includes/MccGenerated/tmr3.c ../Includes/Usb/Framework/usb_device.c ../Includes/Usb/Framework/usb_device_cdc.c ../Includes/Usb/usb_descriptors.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/main.p1 ${OBJECTDIR}/_ext/665382703/poweruptest.p1 ${OBJECTDIR}/_ext/665382703/MOTEapp.p1 ${OBJECTDIR}/_ext/665382703/USBapp.p1 ${OBJECTDIR}/_ext/1909211018/buttons.p1 ${OBJECTDIR}/_ext/1909211018/TMRapp.p1 ${OBJECTDIR}/_ext/1909211018/SSD1306oLED.p1 ${OBJECTDIR}/_ext/2047275575/eusart.p1 ${OBJECTDIR}/_ext/2047275575/interrupt_manager.p1 ${OBJECTDIR}/_ext/2047275575/mcc.p1 ${OBJECTDIR}/_ext/2047275575/adc.p1 ${OBJECTDIR}/_ext/2047275575/spi.p1 ${OBJECTDIR}/_ext/2047275575/tmr1.p1 ${OBJECTDIR}/_ext/2047275575/pin_manager.p1 ${OBJECTDIR}/_ext/2047275575/memory.p1 ${OBJECTDIR}/_ext/2047275575/cmp1.p1 ${OBJECTDIR}/_ext/2047275575/fvr.p1 ${OBJECTDIR}/_ext/2047275575/tmr2.p1 ${OBJECTDIR}/_ext/2047275575/tmr3.p1 ${OBJECTDIR}/_ext/160861694/usb_device.p1 ${OBJECTDIR}/_ext/160861694/usb_device_cdc.p1 ${OBJECTDIR}/_ext/1926928849/usb_descriptors.p1
POSSIBLE_DEPFILES=${OBJECTDIR}/main.p1.d ${OBJECTDIR}/_ext/665382703/poweruptest.p1.d ${OBJECTDIR}/_ext/665382703/MOTEapp.p1.d ${OBJECTDIR}/_ext/665382703/USBapp.p1.d ${OBJECTDIR}/_ext/1909211018/buttons.p1.d ${OBJECTDIR}/_ext/1909211018/TMRapp.p1.d ${OBJECTDIR}/_ext/1909211018/SSD1306oLED.p1.d ${OBJECTDIR}/_ext/2047275575/eusart.p1.d ${OBJECTDIR}/_ext/2047275575/interrupt_manager.p1.d ${OBJECTDIR}/_ext/2047275575/mcc.p1.d ${OBJECTDIR}/_ext/2047275575/adc.p1.d ${OBJECTDIR}/_ext/2047275575/spi.p1.d ${OBJECTDIR}/_ext/2047275575/tmr1.p1.d ${OBJECTDIR}/_ext/2047275575/pin_manager.p1.d ${OBJECTDIR}/_ext/2047275575/memory.p1.d ${OBJECTDIR}/_ext/2047275575/cmp1.p1.d ${OBJECTDIR}/_ext/2047275575/fvr.p1.d ${OBJECTDIR}/_ext/2047275575/tmr2.p1.d ${OBJECTDIR}/_ext/2047275575/tmr3.p1.d ${OBJECTDIR}/_ext/160861694/usb_device.p1.d ${OBJECTDIR}/_ext/160861694/usb_device_cdc.p1.d ${OBJECTDIR}/_ext/1926928849/usb_descriptors.p1.d

# Object Files
OBJECTFILES=${OBJECTDIR}/main.p1 ${OBJECTDIR}/_ext/665382703/poweruptest.p1 ${OBJECTDIR}/_ext/665382703/MOTEapp.p1 ${OBJECTDIR}/_ext/665382703/USBapp.p1 ${OBJECTDIR}/_ext/1909211018/buttons.p1 ${OBJECTDIR}/_ext/1909211018/TMRapp.p1 ${OBJECTDIR}/_ext/1909211018/SSD1306oLED.p1 ${OBJECTDIR}/_ext/2047275575/eusart.p1 ${OBJECTDIR}/_ext/2047275575/interrupt_manager.p1 ${OBJECTDIR}/_ext/2047275575/mcc.p1 ${OBJECTDIR}/_ext/2047275575/adc.p1 ${OBJECTDIR}/_ext/2047275575/spi.p1 ${OBJECTDIR}/_ext/2047275575/tmr1.p1 ${OBJECTDIR}/_ext/2047275575/pin_manager.p1 ${OBJECTDIR}/_ext/2047275575/memory.p1 ${OBJECTDIR}/_ext/2047275575/cmp1.p1 ${OBJECTDIR}/_ext/2047275575/fvr.p1 ${OBJECTDIR}/_ext/2047275575/tmr2.p1 ${OBJECTDIR}/_ext/2047275575/tmr3.p1 ${OBJECTDIR}/_ext/160861694/usb_device.p1 ${OBJECTDIR}/_ext/160861694/usb_device_cdc.p1 ${OBJECTDIR}/_ext/1926928849/usb_descriptors.p1

# Source Files
SOURCEFILES=main.c ../Includes/Board/poweruptest.c ../Includes/Board/MOTEapp.c ../Includes/Board/USBapp.c ../Includes/Drivers/buttons.c ../Includes/Drivers/TMRapp.c ../Includes/Drivers/SSD1306oLED.c ../Includes/MccGenerated/eusart.c ../Includes/MccGenerated/interrupt_manager.c ../Includes/MccGenerated/mcc.c ../Includes/MccGenerated/adc.c ../Includes/MccGenerated/spi.c ../Includes/MccGenerated/tmr1.c ../Includes/MccGenerated/pin_manager.c ../Includes/MccGenerated/memory.c ../Includes/MccGenerated/cmp1.c ../Includes/MccGenerated/fvr.c ../Includes/MccGenerated/tmr2.c ../Includes/MccGenerated/tmr3.c ../Includes/Usb/Framework/usb_device.c ../Includes/Usb/Framework/usb_device_cdc.c ../Includes/Usb/usb_descriptors.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-Standalone.mk dist/${CND_CONF}/${IMAGE_TYPE}/LoRaMote.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18LF45K50
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/main.p1: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.p1.d 
	@${RM} ${OBJECTDIR}/main.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/main.p1  main.c 
	@-${MV} ${OBJECTDIR}/main.d ${OBJECTDIR}/main.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/main.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/665382703/poweruptest.p1: ../Includes/Board/poweruptest.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/665382703" 
	@${RM} ${OBJECTDIR}/_ext/665382703/poweruptest.p1.d 
	@${RM} ${OBJECTDIR}/_ext/665382703/poweruptest.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/665382703/poweruptest.p1  ../Includes/Board/poweruptest.c 
	@-${MV} ${OBJECTDIR}/_ext/665382703/poweruptest.d ${OBJECTDIR}/_ext/665382703/poweruptest.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/665382703/poweruptest.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/665382703/MOTEapp.p1: ../Includes/Board/MOTEapp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/665382703" 
	@${RM} ${OBJECTDIR}/_ext/665382703/MOTEapp.p1.d 
	@${RM} ${OBJECTDIR}/_ext/665382703/MOTEapp.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/665382703/MOTEapp.p1  ../Includes/Board/MOTEapp.c 
	@-${MV} ${OBJECTDIR}/_ext/665382703/MOTEapp.d ${OBJECTDIR}/_ext/665382703/MOTEapp.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/665382703/MOTEapp.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/665382703/USBapp.p1: ../Includes/Board/USBapp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/665382703" 
	@${RM} ${OBJECTDIR}/_ext/665382703/USBapp.p1.d 
	@${RM} ${OBJECTDIR}/_ext/665382703/USBapp.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/665382703/USBapp.p1  ../Includes/Board/USBapp.c 
	@-${MV} ${OBJECTDIR}/_ext/665382703/USBapp.d ${OBJECTDIR}/_ext/665382703/USBapp.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/665382703/USBapp.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1909211018/buttons.p1: ../Includes/Drivers/buttons.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1909211018" 
	@${RM} ${OBJECTDIR}/_ext/1909211018/buttons.p1.d 
	@${RM} ${OBJECTDIR}/_ext/1909211018/buttons.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/1909211018/buttons.p1  ../Includes/Drivers/buttons.c 
	@-${MV} ${OBJECTDIR}/_ext/1909211018/buttons.d ${OBJECTDIR}/_ext/1909211018/buttons.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/1909211018/buttons.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1909211018/TMRapp.p1: ../Includes/Drivers/TMRapp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1909211018" 
	@${RM} ${OBJECTDIR}/_ext/1909211018/TMRapp.p1.d 
	@${RM} ${OBJECTDIR}/_ext/1909211018/TMRapp.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/1909211018/TMRapp.p1  ../Includes/Drivers/TMRapp.c 
	@-${MV} ${OBJECTDIR}/_ext/1909211018/TMRapp.d ${OBJECTDIR}/_ext/1909211018/TMRapp.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/1909211018/TMRapp.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1909211018/SSD1306oLED.p1: ../Includes/Drivers/SSD1306oLED.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1909211018" 
	@${RM} ${OBJECTDIR}/_ext/1909211018/SSD1306oLED.p1.d 
	@${RM} ${OBJECTDIR}/_ext/1909211018/SSD1306oLED.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/1909211018/SSD1306oLED.p1  ../Includes/Drivers/SSD1306oLED.c 
	@-${MV} ${OBJECTDIR}/_ext/1909211018/SSD1306oLED.d ${OBJECTDIR}/_ext/1909211018/SSD1306oLED.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/1909211018/SSD1306oLED.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/eusart.p1: ../Includes/MccGenerated/eusart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/eusart.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/eusart.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/eusart.p1  ../Includes/MccGenerated/eusart.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/eusart.d ${OBJECTDIR}/_ext/2047275575/eusart.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/eusart.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/interrupt_manager.p1: ../Includes/MccGenerated/interrupt_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/interrupt_manager.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/interrupt_manager.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/interrupt_manager.p1  ../Includes/MccGenerated/interrupt_manager.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/interrupt_manager.d ${OBJECTDIR}/_ext/2047275575/interrupt_manager.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/interrupt_manager.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/mcc.p1: ../Includes/MccGenerated/mcc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/mcc.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/mcc.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/mcc.p1  ../Includes/MccGenerated/mcc.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/mcc.d ${OBJECTDIR}/_ext/2047275575/mcc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/mcc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/adc.p1: ../Includes/MccGenerated/adc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/adc.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/adc.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/adc.p1  ../Includes/MccGenerated/adc.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/adc.d ${OBJECTDIR}/_ext/2047275575/adc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/adc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/spi.p1: ../Includes/MccGenerated/spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/spi.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/spi.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/spi.p1  ../Includes/MccGenerated/spi.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/spi.d ${OBJECTDIR}/_ext/2047275575/spi.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/spi.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/tmr1.p1: ../Includes/MccGenerated/tmr1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/tmr1.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/tmr1.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/tmr1.p1  ../Includes/MccGenerated/tmr1.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/tmr1.d ${OBJECTDIR}/_ext/2047275575/tmr1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/tmr1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/pin_manager.p1: ../Includes/MccGenerated/pin_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/pin_manager.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/pin_manager.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/pin_manager.p1  ../Includes/MccGenerated/pin_manager.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/pin_manager.d ${OBJECTDIR}/_ext/2047275575/pin_manager.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/pin_manager.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/memory.p1: ../Includes/MccGenerated/memory.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/memory.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/memory.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/memory.p1  ../Includes/MccGenerated/memory.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/memory.d ${OBJECTDIR}/_ext/2047275575/memory.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/memory.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/cmp1.p1: ../Includes/MccGenerated/cmp1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/cmp1.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/cmp1.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/cmp1.p1  ../Includes/MccGenerated/cmp1.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/cmp1.d ${OBJECTDIR}/_ext/2047275575/cmp1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/cmp1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/fvr.p1: ../Includes/MccGenerated/fvr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/fvr.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/fvr.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/fvr.p1  ../Includes/MccGenerated/fvr.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/fvr.d ${OBJECTDIR}/_ext/2047275575/fvr.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/fvr.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/tmr2.p1: ../Includes/MccGenerated/tmr2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/tmr2.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/tmr2.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/tmr2.p1  ../Includes/MccGenerated/tmr2.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/tmr2.d ${OBJECTDIR}/_ext/2047275575/tmr2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/tmr2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/tmr3.p1: ../Includes/MccGenerated/tmr3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/tmr3.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/tmr3.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/tmr3.p1  ../Includes/MccGenerated/tmr3.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/tmr3.d ${OBJECTDIR}/_ext/2047275575/tmr3.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/tmr3.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/160861694/usb_device.p1: ../Includes/Usb/Framework/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/160861694" 
	@${RM} ${OBJECTDIR}/_ext/160861694/usb_device.p1.d 
	@${RM} ${OBJECTDIR}/_ext/160861694/usb_device.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/160861694/usb_device.p1  ../Includes/Usb/Framework/usb_device.c 
	@-${MV} ${OBJECTDIR}/_ext/160861694/usb_device.d ${OBJECTDIR}/_ext/160861694/usb_device.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/160861694/usb_device.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/160861694/usb_device_cdc.p1: ../Includes/Usb/Framework/usb_device_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/160861694" 
	@${RM} ${OBJECTDIR}/_ext/160861694/usb_device_cdc.p1.d 
	@${RM} ${OBJECTDIR}/_ext/160861694/usb_device_cdc.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/160861694/usb_device_cdc.p1  ../Includes/Usb/Framework/usb_device_cdc.c 
	@-${MV} ${OBJECTDIR}/_ext/160861694/usb_device_cdc.d ${OBJECTDIR}/_ext/160861694/usb_device_cdc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/160861694/usb_device_cdc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1926928849/usb_descriptors.p1: ../Includes/Usb/usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1926928849" 
	@${RM} ${OBJECTDIR}/_ext/1926928849/usb_descriptors.p1.d 
	@${RM} ${OBJECTDIR}/_ext/1926928849/usb_descriptors.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/1926928849/usb_descriptors.p1  ../Includes/Usb/usb_descriptors.c 
	@-${MV} ${OBJECTDIR}/_ext/1926928849/usb_descriptors.d ${OBJECTDIR}/_ext/1926928849/usb_descriptors.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/1926928849/usb_descriptors.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/main.p1: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.p1.d 
	@${RM} ${OBJECTDIR}/main.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/main.p1  main.c 
	@-${MV} ${OBJECTDIR}/main.d ${OBJECTDIR}/main.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/main.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/665382703/poweruptest.p1: ../Includes/Board/poweruptest.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/665382703" 
	@${RM} ${OBJECTDIR}/_ext/665382703/poweruptest.p1.d 
	@${RM} ${OBJECTDIR}/_ext/665382703/poweruptest.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/665382703/poweruptest.p1  ../Includes/Board/poweruptest.c 
	@-${MV} ${OBJECTDIR}/_ext/665382703/poweruptest.d ${OBJECTDIR}/_ext/665382703/poweruptest.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/665382703/poweruptest.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/665382703/MOTEapp.p1: ../Includes/Board/MOTEapp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/665382703" 
	@${RM} ${OBJECTDIR}/_ext/665382703/MOTEapp.p1.d 
	@${RM} ${OBJECTDIR}/_ext/665382703/MOTEapp.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/665382703/MOTEapp.p1  ../Includes/Board/MOTEapp.c 
	@-${MV} ${OBJECTDIR}/_ext/665382703/MOTEapp.d ${OBJECTDIR}/_ext/665382703/MOTEapp.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/665382703/MOTEapp.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/665382703/USBapp.p1: ../Includes/Board/USBapp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/665382703" 
	@${RM} ${OBJECTDIR}/_ext/665382703/USBapp.p1.d 
	@${RM} ${OBJECTDIR}/_ext/665382703/USBapp.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/665382703/USBapp.p1  ../Includes/Board/USBapp.c 
	@-${MV} ${OBJECTDIR}/_ext/665382703/USBapp.d ${OBJECTDIR}/_ext/665382703/USBapp.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/665382703/USBapp.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1909211018/buttons.p1: ../Includes/Drivers/buttons.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1909211018" 
	@${RM} ${OBJECTDIR}/_ext/1909211018/buttons.p1.d 
	@${RM} ${OBJECTDIR}/_ext/1909211018/buttons.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/1909211018/buttons.p1  ../Includes/Drivers/buttons.c 
	@-${MV} ${OBJECTDIR}/_ext/1909211018/buttons.d ${OBJECTDIR}/_ext/1909211018/buttons.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/1909211018/buttons.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1909211018/TMRapp.p1: ../Includes/Drivers/TMRapp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1909211018" 
	@${RM} ${OBJECTDIR}/_ext/1909211018/TMRapp.p1.d 
	@${RM} ${OBJECTDIR}/_ext/1909211018/TMRapp.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/1909211018/TMRapp.p1  ../Includes/Drivers/TMRapp.c 
	@-${MV} ${OBJECTDIR}/_ext/1909211018/TMRapp.d ${OBJECTDIR}/_ext/1909211018/TMRapp.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/1909211018/TMRapp.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1909211018/SSD1306oLED.p1: ../Includes/Drivers/SSD1306oLED.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1909211018" 
	@${RM} ${OBJECTDIR}/_ext/1909211018/SSD1306oLED.p1.d 
	@${RM} ${OBJECTDIR}/_ext/1909211018/SSD1306oLED.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/1909211018/SSD1306oLED.p1  ../Includes/Drivers/SSD1306oLED.c 
	@-${MV} ${OBJECTDIR}/_ext/1909211018/SSD1306oLED.d ${OBJECTDIR}/_ext/1909211018/SSD1306oLED.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/1909211018/SSD1306oLED.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/eusart.p1: ../Includes/MccGenerated/eusart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/eusart.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/eusart.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/eusart.p1  ../Includes/MccGenerated/eusart.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/eusart.d ${OBJECTDIR}/_ext/2047275575/eusart.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/eusart.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/interrupt_manager.p1: ../Includes/MccGenerated/interrupt_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/interrupt_manager.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/interrupt_manager.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/interrupt_manager.p1  ../Includes/MccGenerated/interrupt_manager.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/interrupt_manager.d ${OBJECTDIR}/_ext/2047275575/interrupt_manager.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/interrupt_manager.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/mcc.p1: ../Includes/MccGenerated/mcc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/mcc.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/mcc.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/mcc.p1  ../Includes/MccGenerated/mcc.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/mcc.d ${OBJECTDIR}/_ext/2047275575/mcc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/mcc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/adc.p1: ../Includes/MccGenerated/adc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/adc.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/adc.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/adc.p1  ../Includes/MccGenerated/adc.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/adc.d ${OBJECTDIR}/_ext/2047275575/adc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/adc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/spi.p1: ../Includes/MccGenerated/spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/spi.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/spi.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/spi.p1  ../Includes/MccGenerated/spi.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/spi.d ${OBJECTDIR}/_ext/2047275575/spi.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/spi.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/tmr1.p1: ../Includes/MccGenerated/tmr1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/tmr1.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/tmr1.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/tmr1.p1  ../Includes/MccGenerated/tmr1.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/tmr1.d ${OBJECTDIR}/_ext/2047275575/tmr1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/tmr1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/pin_manager.p1: ../Includes/MccGenerated/pin_manager.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/pin_manager.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/pin_manager.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/pin_manager.p1  ../Includes/MccGenerated/pin_manager.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/pin_manager.d ${OBJECTDIR}/_ext/2047275575/pin_manager.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/pin_manager.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/memory.p1: ../Includes/MccGenerated/memory.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/memory.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/memory.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/memory.p1  ../Includes/MccGenerated/memory.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/memory.d ${OBJECTDIR}/_ext/2047275575/memory.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/memory.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/cmp1.p1: ../Includes/MccGenerated/cmp1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/cmp1.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/cmp1.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/cmp1.p1  ../Includes/MccGenerated/cmp1.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/cmp1.d ${OBJECTDIR}/_ext/2047275575/cmp1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/cmp1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/fvr.p1: ../Includes/MccGenerated/fvr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/fvr.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/fvr.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/fvr.p1  ../Includes/MccGenerated/fvr.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/fvr.d ${OBJECTDIR}/_ext/2047275575/fvr.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/fvr.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/tmr2.p1: ../Includes/MccGenerated/tmr2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/tmr2.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/tmr2.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/tmr2.p1  ../Includes/MccGenerated/tmr2.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/tmr2.d ${OBJECTDIR}/_ext/2047275575/tmr2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/tmr2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/2047275575/tmr3.p1: ../Includes/MccGenerated/tmr3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2047275575" 
	@${RM} ${OBJECTDIR}/_ext/2047275575/tmr3.p1.d 
	@${RM} ${OBJECTDIR}/_ext/2047275575/tmr3.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/2047275575/tmr3.p1  ../Includes/MccGenerated/tmr3.c 
	@-${MV} ${OBJECTDIR}/_ext/2047275575/tmr3.d ${OBJECTDIR}/_ext/2047275575/tmr3.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/2047275575/tmr3.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/160861694/usb_device.p1: ../Includes/Usb/Framework/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/160861694" 
	@${RM} ${OBJECTDIR}/_ext/160861694/usb_device.p1.d 
	@${RM} ${OBJECTDIR}/_ext/160861694/usb_device.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/160861694/usb_device.p1  ../Includes/Usb/Framework/usb_device.c 
	@-${MV} ${OBJECTDIR}/_ext/160861694/usb_device.d ${OBJECTDIR}/_ext/160861694/usb_device.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/160861694/usb_device.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/160861694/usb_device_cdc.p1: ../Includes/Usb/Framework/usb_device_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/160861694" 
	@${RM} ${OBJECTDIR}/_ext/160861694/usb_device_cdc.p1.d 
	@${RM} ${OBJECTDIR}/_ext/160861694/usb_device_cdc.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/160861694/usb_device_cdc.p1  ../Includes/Usb/Framework/usb_device_cdc.c 
	@-${MV} ${OBJECTDIR}/_ext/160861694/usb_device_cdc.d ${OBJECTDIR}/_ext/160861694/usb_device_cdc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/160861694/usb_device_cdc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/_ext/1926928849/usb_descriptors.p1: ../Includes/Usb/usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1926928849" 
	@${RM} ${OBJECTDIR}/_ext/1926928849/usb_descriptors.p1.d 
	@${RM} ${OBJECTDIR}/_ext/1926928849/usb_descriptors.p1 
	${MP_CC} --pass1 $(MP_EXTRA_CC_PRE) --chip=$(MP_PROCESSOR_OPTION) -Q -G  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    -o${OBJECTDIR}/_ext/1926928849/usb_descriptors.p1  ../Includes/Usb/usb_descriptors.c 
	@-${MV} ${OBJECTDIR}/_ext/1926928849/usb_descriptors.d ${OBJECTDIR}/_ext/1926928849/usb_descriptors.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/_ext/1926928849/usb_descriptors.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/LoRaMote.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) --chip=$(MP_PROCESSOR_OPTION) -G -mdist/${CND_CONF}/${IMAGE_TYPE}/LoRaMote.X.${IMAGE_TYPE}.map  -D__DEBUG=1 --debugger=icd3  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"       --memorysummary dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -odist/${CND_CONF}/${IMAGE_TYPE}/LoRaMote.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	@${RM} dist/${CND_CONF}/${IMAGE_TYPE}/LoRaMote.X.${IMAGE_TYPE}.hex 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/LoRaMote.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) --chip=$(MP_PROCESSOR_OPTION) -G -mdist/${CND_CONF}/${IMAGE_TYPE}/LoRaMote.X.${IMAGE_TYPE}.map  --double=32 --float=32 --emi=wordwrite --opt=default,+asm,-asmfile,+speed,-space,+debug --addrqual=ignore --mode=pro -P -N255 -I"../Includes/MccGenerated" -I"../Includes/Drivers" -I"../Includes/Board" -I"../Includes/Usb" -I"../Includes/Usb/Framework" --warn=0 --asmlist --summary=default,-psect,-class,+mem,-hex,-file --output=default,-inhx032 --runtime=default,+clear,+init,-keep,-no_startup,-download,+config,+clib,+plib --output=-mcof,+elf:multilocs --stack=compiled:auto:auto:auto "--errformat=%f:%l: error: (%n) %s" "--warnformat=%f:%l: warning: (%n) %s" "--msgformat=%f:%l: advisory: (%n) %s"    --memorysummary dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -odist/${CND_CONF}/${IMAGE_TYPE}/LoRaMote.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/Standalone
	${RM} -r dist/Standalone

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
