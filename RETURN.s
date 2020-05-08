
   .def push
   .def pop
   .def setPSP
   .def setStack_pointer
   .def setStack_pointer2
   .def setStack_pointer3
   .def testValues1_R11_to_R4
   .def testValues2_R4_to_R11
   .def get_R0

.thumb
.const

.text

push:

   MRS R0,PSP 
   SUB R0,R0,#04
   STR R11,[R0]
   MSR PSP,R0 
   MRS R0,PSP 
   SUB R0,R0,#04 
   STR R10,[R0]
   MSR PSP,R0 
   MRS R0,PSP 
   SUB R0,R0,#04
   STR R9,[R0]
   MSR PSP,R0 
   MRS R0,PSP 
   SUB R0,R0,#04
   STR R8,[R0]
   MSR PSP,R0 
   MRS R0,PSP 
   SUB R0,R0,#04
   STR R7,[R0]
   MSR PSP,R0 
   MRS R0,PSP 
   SUB R0,R0,#04
   STR R6,[R0]
   MSR PSP,R0 
   MRS R0,PSP 
   SUB R0,R0,#04 
   STR R5,[R0]
   MSR PSP,R0 
   MRS R0,PSP 
   SUB R0,R0,#04
   STR R4,[R0]
   MSR PSP,R0
   BX LR

pop:
   MRS R0,PSP
   LDR R4,[R0]
   ADD R0,R0,#04
   MSR PSP,R0
   MRS R0,PSP
   LDR R5,[R0]
   ADD R0,R0,#04
   MSR PSP,R0
   MRS R0,PSP
   LDR R6,[R0]
   ADD R0,R0,#04
   MSR PSP,R0
   MRS R0,PSP
   LDR R7,[R0]
   ADD R0,R0,#04
   MSR PSP,R0
   MRS R0,PSP
   LDR R8,[R0]
   ADD R0,R0,#04
   MSR PSP,R0
   MRS R0,PSP
   LDR R9,[R0]
   ADD R0,R0,#04
   MSR PSP,R0
   MRS R0,PSP
   LDR R10,[R0]
   ADD R0,R0,#04
   MSR PSP,R0
   MRS R0,PSP
   LDR R11,[R0]
   ADD R0,R0,#04
   MSR PSP,R0
   BX LR

setPSP:
 MSR PSP,R0
 BX LR

setStack_pointer:
     MOV R3, #02
     MSR CONTROL, R3
     ISB
     MRS SP,MSP
     BX LR

setStack_pointer2:
     MRS R0,PSP
     BX LR

setStack_pointer3:
     MRS R1,PSP
     BX LR


testValues1_R11_to_R4:
         MOV R11,#5
         MOV R10,#6
         MOV R9,#7
         MOV R8,#8
         MOV R7,#9
         MOV R6,#10
         MOV R5,#11
         MOV R4,#12
         BX LR

testValues2_R4_to_R11:
 MOV R4,#3
 MOV R5,#3
 MOV R6,#3
 MOV R7,#3
 MOV R8,#3
 MOV R9,#3
 MOV R10,#3
 MOV R11,#3
 BX LR

get_R0:
 MOV R1,R0
 MOV R0,R1
 BX LR

.endm
