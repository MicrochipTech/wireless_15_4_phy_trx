/*
 * Component description for DSU
 *
 * Copyright (c) 2022 Microchip Technology Inc. and its subsidiaries.
 *
 * Subject to your compliance with these terms, you may use Microchip software and any derivatives
 * exclusively with Microchip products. It is your responsibility to comply with third party license
 * terms applicable to your use of third party software (including open source software) that may
 * accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 * APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND
 * FITNESS FOR A PARTICULAR PURPOSE.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF
 * MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT
 * EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

/* file generated from device description version 2020-11-26T16:56:39Z */
#ifndef _SAMR21_DSU_COMPONENT_H_
#define _SAMR21_DSU_COMPONENT_H_

/* ************************************************************************** */
/*   SOFTWARE API DEFINITION FOR DSU                                          */
/* ************************************************************************** */

/* -------- DSU_CTRL : (DSU Offset: 0x00) ( /W 8) Control -------- */
#define DSU_CTRL_RESETVALUE                   _UINT8_(0x00)                                        /*  (DSU_CTRL) Control  Reset Value */

#define DSU_CTRL_SWRST_Pos                    _UINT8_(0)                                           /* (DSU_CTRL) Software Reset Position */
#define DSU_CTRL_SWRST_Msk                    (_UINT8_(0x1) << DSU_CTRL_SWRST_Pos)                 /* (DSU_CTRL) Software Reset Mask */
#define DSU_CTRL_SWRST(value)                 (DSU_CTRL_SWRST_Msk & (_UINT8_(value) << DSU_CTRL_SWRST_Pos)) /* Assigment of value for SWRST in the DSU_CTRL register */
#define DSU_CTRL_CRC_Pos                      _UINT8_(2)                                           /* (DSU_CTRL) 32-bit Cyclic Redundancy Check Position */
#define DSU_CTRL_CRC_Msk                      (_UINT8_(0x1) << DSU_CTRL_CRC_Pos)                   /* (DSU_CTRL) 32-bit Cyclic Redundancy Check Mask */
#define DSU_CTRL_CRC(value)                   (DSU_CTRL_CRC_Msk & (_UINT8_(value) << DSU_CTRL_CRC_Pos)) /* Assigment of value for CRC in the DSU_CTRL register */
#define DSU_CTRL_MBIST_Pos                    _UINT8_(3)                                           /* (DSU_CTRL) Memory Built-In Self-Test Position */
#define DSU_CTRL_MBIST_Msk                    (_UINT8_(0x1) << DSU_CTRL_MBIST_Pos)                 /* (DSU_CTRL) Memory Built-In Self-Test Mask */
#define DSU_CTRL_MBIST(value)                 (DSU_CTRL_MBIST_Msk & (_UINT8_(value) << DSU_CTRL_MBIST_Pos)) /* Assigment of value for MBIST in the DSU_CTRL register */
#define DSU_CTRL_CE_Pos                       _UINT8_(4)                                           /* (DSU_CTRL) Chip Erase Position */
#define DSU_CTRL_CE_Msk                       (_UINT8_(0x1) << DSU_CTRL_CE_Pos)                    /* (DSU_CTRL) Chip Erase Mask */
#define DSU_CTRL_CE(value)                    (DSU_CTRL_CE_Msk & (_UINT8_(value) << DSU_CTRL_CE_Pos)) /* Assigment of value for CE in the DSU_CTRL register */
#define DSU_CTRL_Msk                          _UINT8_(0x1D)                                        /* (DSU_CTRL) Register Mask  */


/* -------- DSU_STATUSA : (DSU Offset: 0x01) (R/W 8) Status A -------- */
#define DSU_STATUSA_RESETVALUE                _UINT8_(0x00)                                        /*  (DSU_STATUSA) Status A  Reset Value */

#define DSU_STATUSA_DONE_Pos                  _UINT8_(0)                                           /* (DSU_STATUSA) Done Position */
#define DSU_STATUSA_DONE_Msk                  (_UINT8_(0x1) << DSU_STATUSA_DONE_Pos)               /* (DSU_STATUSA) Done Mask */
#define DSU_STATUSA_DONE(value)               (DSU_STATUSA_DONE_Msk & (_UINT8_(value) << DSU_STATUSA_DONE_Pos)) /* Assigment of value for DONE in the DSU_STATUSA register */
#define DSU_STATUSA_CRSTEXT_Pos               _UINT8_(1)                                           /* (DSU_STATUSA) CPU Reset Phase Extension Position */
#define DSU_STATUSA_CRSTEXT_Msk               (_UINT8_(0x1) << DSU_STATUSA_CRSTEXT_Pos)            /* (DSU_STATUSA) CPU Reset Phase Extension Mask */
#define DSU_STATUSA_CRSTEXT(value)            (DSU_STATUSA_CRSTEXT_Msk & (_UINT8_(value) << DSU_STATUSA_CRSTEXT_Pos)) /* Assigment of value for CRSTEXT in the DSU_STATUSA register */
#define DSU_STATUSA_BERR_Pos                  _UINT8_(2)                                           /* (DSU_STATUSA) Bus Error Position */
#define DSU_STATUSA_BERR_Msk                  (_UINT8_(0x1) << DSU_STATUSA_BERR_Pos)               /* (DSU_STATUSA) Bus Error Mask */
#define DSU_STATUSA_BERR(value)               (DSU_STATUSA_BERR_Msk & (_UINT8_(value) << DSU_STATUSA_BERR_Pos)) /* Assigment of value for BERR in the DSU_STATUSA register */
#define DSU_STATUSA_FAIL_Pos                  _UINT8_(3)                                           /* (DSU_STATUSA) Failure Position */
#define DSU_STATUSA_FAIL_Msk                  (_UINT8_(0x1) << DSU_STATUSA_FAIL_Pos)               /* (DSU_STATUSA) Failure Mask */
#define DSU_STATUSA_FAIL(value)               (DSU_STATUSA_FAIL_Msk & (_UINT8_(value) << DSU_STATUSA_FAIL_Pos)) /* Assigment of value for FAIL in the DSU_STATUSA register */
#define DSU_STATUSA_PERR_Pos                  _UINT8_(4)                                           /* (DSU_STATUSA) Protection Error Position */
#define DSU_STATUSA_PERR_Msk                  (_UINT8_(0x1) << DSU_STATUSA_PERR_Pos)               /* (DSU_STATUSA) Protection Error Mask */
#define DSU_STATUSA_PERR(value)               (DSU_STATUSA_PERR_Msk & (_UINT8_(value) << DSU_STATUSA_PERR_Pos)) /* Assigment of value for PERR in the DSU_STATUSA register */
#define DSU_STATUSA_Msk                       _UINT8_(0x1F)                                        /* (DSU_STATUSA) Register Mask  */


/* -------- DSU_STATUSB : (DSU Offset: 0x02) ( R/ 8) Status B -------- */
#define DSU_STATUSB_RESETVALUE                _UINT8_(0x10)                                        /*  (DSU_STATUSB) Status B  Reset Value */

#define DSU_STATUSB_PROT_Pos                  _UINT8_(0)                                           /* (DSU_STATUSB) Protected Position */
#define DSU_STATUSB_PROT_Msk                  (_UINT8_(0x1) << DSU_STATUSB_PROT_Pos)               /* (DSU_STATUSB) Protected Mask */
#define DSU_STATUSB_PROT(value)               (DSU_STATUSB_PROT_Msk & (_UINT8_(value) << DSU_STATUSB_PROT_Pos)) /* Assigment of value for PROT in the DSU_STATUSB register */
#define DSU_STATUSB_DBGPRES_Pos               _UINT8_(1)                                           /* (DSU_STATUSB) Debugger Present Position */
#define DSU_STATUSB_DBGPRES_Msk               (_UINT8_(0x1) << DSU_STATUSB_DBGPRES_Pos)            /* (DSU_STATUSB) Debugger Present Mask */
#define DSU_STATUSB_DBGPRES(value)            (DSU_STATUSB_DBGPRES_Msk & (_UINT8_(value) << DSU_STATUSB_DBGPRES_Pos)) /* Assigment of value for DBGPRES in the DSU_STATUSB register */
#define DSU_STATUSB_DCCD0_Pos                 _UINT8_(2)                                           /* (DSU_STATUSB) Debug Communication Channel 0 Dirty Position */
#define DSU_STATUSB_DCCD0_Msk                 (_UINT8_(0x1) << DSU_STATUSB_DCCD0_Pos)              /* (DSU_STATUSB) Debug Communication Channel 0 Dirty Mask */
#define DSU_STATUSB_DCCD0(value)              (DSU_STATUSB_DCCD0_Msk & (_UINT8_(value) << DSU_STATUSB_DCCD0_Pos)) /* Assigment of value for DCCD0 in the DSU_STATUSB register */
#define DSU_STATUSB_DCCD1_Pos                 _UINT8_(3)                                           /* (DSU_STATUSB) Debug Communication Channel 1 Dirty Position */
#define DSU_STATUSB_DCCD1_Msk                 (_UINT8_(0x1) << DSU_STATUSB_DCCD1_Pos)              /* (DSU_STATUSB) Debug Communication Channel 1 Dirty Mask */
#define DSU_STATUSB_DCCD1(value)              (DSU_STATUSB_DCCD1_Msk & (_UINT8_(value) << DSU_STATUSB_DCCD1_Pos)) /* Assigment of value for DCCD1 in the DSU_STATUSB register */
#define DSU_STATUSB_HPE_Pos                   _UINT8_(4)                                           /* (DSU_STATUSB) Hot-Plugging Enable Position */
#define DSU_STATUSB_HPE_Msk                   (_UINT8_(0x1) << DSU_STATUSB_HPE_Pos)                /* (DSU_STATUSB) Hot-Plugging Enable Mask */
#define DSU_STATUSB_HPE(value)                (DSU_STATUSB_HPE_Msk & (_UINT8_(value) << DSU_STATUSB_HPE_Pos)) /* Assigment of value for HPE in the DSU_STATUSB register */
#define DSU_STATUSB_Msk                       _UINT8_(0x1F)                                        /* (DSU_STATUSB) Register Mask  */

#define DSU_STATUSB_DCCD_Pos                  _UINT8_(2)                                           /* (DSU_STATUSB Position) Debug Communication Channel x Dirty */
#define DSU_STATUSB_DCCD_Msk                  (_UINT8_(0x3) << DSU_STATUSB_DCCD_Pos)               /* (DSU_STATUSB Mask) DCCD */
#define DSU_STATUSB_DCCD(value)               (DSU_STATUSB_DCCD_Msk & (_UINT8_(value) << DSU_STATUSB_DCCD_Pos)) 

/* -------- DSU_ADDR : (DSU Offset: 0x04) (R/W 32) Address -------- */
#define DSU_ADDR_RESETVALUE                   _UINT32_(0x00)                                       /*  (DSU_ADDR) Address  Reset Value */

#define DSU_ADDR_ADDR_Pos                     _UINT32_(2)                                          /* (DSU_ADDR) Address Position */
#define DSU_ADDR_ADDR_Msk                     (_UINT32_(0x3FFFFFFF) << DSU_ADDR_ADDR_Pos)          /* (DSU_ADDR) Address Mask */
#define DSU_ADDR_ADDR(value)                  (DSU_ADDR_ADDR_Msk & (_UINT32_(value) << DSU_ADDR_ADDR_Pos)) /* Assigment of value for ADDR in the DSU_ADDR register */
#define DSU_ADDR_Msk                          _UINT32_(0xFFFFFFFC)                                 /* (DSU_ADDR) Register Mask  */


/* -------- DSU_LENGTH : (DSU Offset: 0x08) (R/W 32) Length -------- */
#define DSU_LENGTH_RESETVALUE                 _UINT32_(0x00)                                       /*  (DSU_LENGTH) Length  Reset Value */

#define DSU_LENGTH_LENGTH_Pos                 _UINT32_(2)                                          /* (DSU_LENGTH) Length Position */
#define DSU_LENGTH_LENGTH_Msk                 (_UINT32_(0x3FFFFFFF) << DSU_LENGTH_LENGTH_Pos)      /* (DSU_LENGTH) Length Mask */
#define DSU_LENGTH_LENGTH(value)              (DSU_LENGTH_LENGTH_Msk & (_UINT32_(value) << DSU_LENGTH_LENGTH_Pos)) /* Assigment of value for LENGTH in the DSU_LENGTH register */
#define DSU_LENGTH_Msk                        _UINT32_(0xFFFFFFFC)                                 /* (DSU_LENGTH) Register Mask  */


/* -------- DSU_DATA : (DSU Offset: 0x0C) (R/W 32) Data -------- */
#define DSU_DATA_RESETVALUE                   _UINT32_(0x00)                                       /*  (DSU_DATA) Data  Reset Value */

#define DSU_DATA_DATA_Pos                     _UINT32_(0)                                          /* (DSU_DATA) Data Position */
#define DSU_DATA_DATA_Msk                     (_UINT32_(0xFFFFFFFF) << DSU_DATA_DATA_Pos)          /* (DSU_DATA) Data Mask */
#define DSU_DATA_DATA(value)                  (DSU_DATA_DATA_Msk & (_UINT32_(value) << DSU_DATA_DATA_Pos)) /* Assigment of value for DATA in the DSU_DATA register */
#define DSU_DATA_Msk                          _UINT32_(0xFFFFFFFF)                                 /* (DSU_DATA) Register Mask  */


/* -------- DSU_DCC : (DSU Offset: 0x10) (R/W 32) Debug Communication Channel n -------- */
#define DSU_DCC_RESETVALUE                    _UINT32_(0x00)                                       /*  (DSU_DCC) Debug Communication Channel n  Reset Value */

#define DSU_DCC_DATA_Pos                      _UINT32_(0)                                          /* (DSU_DCC) Data Position */
#define DSU_DCC_DATA_Msk                      (_UINT32_(0xFFFFFFFF) << DSU_DCC_DATA_Pos)           /* (DSU_DCC) Data Mask */
#define DSU_DCC_DATA(value)                   (DSU_DCC_DATA_Msk & (_UINT32_(value) << DSU_DCC_DATA_Pos)) /* Assigment of value for DATA in the DSU_DCC register */
#define DSU_DCC_Msk                           _UINT32_(0xFFFFFFFF)                                 /* (DSU_DCC) Register Mask  */


/* -------- DSU_DID : (DSU Offset: 0x18) ( R/ 32) Device Identification -------- */
#define DSU_DID_RESETVALUE                    _UINT32_(0x10010319)                                 /*  (DSU_DID) Device Identification  Reset Value */

#define DSU_DID_DEVSEL_Pos                    _UINT32_(0)                                          /* (DSU_DID) Device Select Position */
#define DSU_DID_DEVSEL_Msk                    (_UINT32_(0xFF) << DSU_DID_DEVSEL_Pos)               /* (DSU_DID) Device Select Mask */
#define DSU_DID_DEVSEL(value)                 (DSU_DID_DEVSEL_Msk & (_UINT32_(value) << DSU_DID_DEVSEL_Pos)) /* Assigment of value for DEVSEL in the DSU_DID register */
#define DSU_DID_REVISION_Pos                  _UINT32_(8)                                          /* (DSU_DID) Revision Position */
#define DSU_DID_REVISION_Msk                  (_UINT32_(0xF) << DSU_DID_REVISION_Pos)              /* (DSU_DID) Revision Mask */
#define DSU_DID_REVISION(value)               (DSU_DID_REVISION_Msk & (_UINT32_(value) << DSU_DID_REVISION_Pos)) /* Assigment of value for REVISION in the DSU_DID register */
#define DSU_DID_DIE_Pos                       _UINT32_(12)                                         /* (DSU_DID) Die Identification Position */
#define DSU_DID_DIE_Msk                       (_UINT32_(0xF) << DSU_DID_DIE_Pos)                   /* (DSU_DID) Die Identification Mask */
#define DSU_DID_DIE(value)                    (DSU_DID_DIE_Msk & (_UINT32_(value) << DSU_DID_DIE_Pos)) /* Assigment of value for DIE in the DSU_DID register */
#define DSU_DID_SERIES_Pos                    _UINT32_(16)                                         /* (DSU_DID) Product Series Position */
#define DSU_DID_SERIES_Msk                    (_UINT32_(0x3F) << DSU_DID_SERIES_Pos)               /* (DSU_DID) Product Series Mask */
#define DSU_DID_SERIES(value)                 (DSU_DID_SERIES_Msk & (_UINT32_(value) << DSU_DID_SERIES_Pos)) /* Assigment of value for SERIES in the DSU_DID register */
#define DSU_DID_FAMILY_Pos                    _UINT32_(23)                                         /* (DSU_DID) Product Family Position */
#define DSU_DID_FAMILY_Msk                    (_UINT32_(0x1F) << DSU_DID_FAMILY_Pos)               /* (DSU_DID) Product Family Mask */
#define DSU_DID_FAMILY(value)                 (DSU_DID_FAMILY_Msk & (_UINT32_(value) << DSU_DID_FAMILY_Pos)) /* Assigment of value for FAMILY in the DSU_DID register */
#define DSU_DID_PROCESSOR_Pos                 _UINT32_(28)                                         /* (DSU_DID) Processor Position */
#define DSU_DID_PROCESSOR_Msk                 (_UINT32_(0xF) << DSU_DID_PROCESSOR_Pos)             /* (DSU_DID) Processor Mask */
#define DSU_DID_PROCESSOR(value)              (DSU_DID_PROCESSOR_Msk & (_UINT32_(value) << DSU_DID_PROCESSOR_Pos)) /* Assigment of value for PROCESSOR in the DSU_DID register */
#define DSU_DID_Msk                           _UINT32_(0xFFBFFFFF)                                 /* (DSU_DID) Register Mask  */


/* -------- DSU_ENTRY0 : (DSU Offset: 0x1000) ( R/ 32) CoreSight ROM Table Entry 0 -------- */
#define DSU_ENTRY0_RESETVALUE                 _UINT32_(0x9F9FC002)                                 /*  (DSU_ENTRY0) CoreSight ROM Table Entry 0  Reset Value */

#define DSU_ENTRY0_EPRES_Pos                  _UINT32_(0)                                          /* (DSU_ENTRY0) Entry Present Position */
#define DSU_ENTRY0_EPRES_Msk                  (_UINT32_(0x1) << DSU_ENTRY0_EPRES_Pos)              /* (DSU_ENTRY0) Entry Present Mask */
#define DSU_ENTRY0_EPRES(value)               (DSU_ENTRY0_EPRES_Msk & (_UINT32_(value) << DSU_ENTRY0_EPRES_Pos)) /* Assigment of value for EPRES in the DSU_ENTRY0 register */
#define DSU_ENTRY0_FMT_Pos                    _UINT32_(1)                                          /* (DSU_ENTRY0) Format Position */
#define DSU_ENTRY0_FMT_Msk                    (_UINT32_(0x1) << DSU_ENTRY0_FMT_Pos)                /* (DSU_ENTRY0) Format Mask */
#define DSU_ENTRY0_FMT(value)                 (DSU_ENTRY0_FMT_Msk & (_UINT32_(value) << DSU_ENTRY0_FMT_Pos)) /* Assigment of value for FMT in the DSU_ENTRY0 register */
#define DSU_ENTRY0_ADDOFF_Pos                 _UINT32_(12)                                         /* (DSU_ENTRY0) Address Offset Position */
#define DSU_ENTRY0_ADDOFF_Msk                 (_UINT32_(0xFFFFF) << DSU_ENTRY0_ADDOFF_Pos)         /* (DSU_ENTRY0) Address Offset Mask */
#define DSU_ENTRY0_ADDOFF(value)              (DSU_ENTRY0_ADDOFF_Msk & (_UINT32_(value) << DSU_ENTRY0_ADDOFF_Pos)) /* Assigment of value for ADDOFF in the DSU_ENTRY0 register */
#define DSU_ENTRY0_Msk                        _UINT32_(0xFFFFF003)                                 /* (DSU_ENTRY0) Register Mask  */


/* -------- DSU_ENTRY1 : (DSU Offset: 0x1004) ( R/ 32) CoreSight ROM Table Entry 1 -------- */
#define DSU_ENTRY1_RESETVALUE                 _UINT32_(0x3002)                                     /*  (DSU_ENTRY1) CoreSight ROM Table Entry 1  Reset Value */

#define DSU_ENTRY1_Msk                        _UINT32_(0x00000000)                                 /* (DSU_ENTRY1) Register Mask  */


/* -------- DSU_END : (DSU Offset: 0x1008) ( R/ 32) CoreSight ROM Table End -------- */
#define DSU_END_RESETVALUE                    _UINT32_(0x00)                                       /*  (DSU_END) CoreSight ROM Table End  Reset Value */

#define DSU_END_END_Pos                       _UINT32_(0)                                          /* (DSU_END) End Marker Position */
#define DSU_END_END_Msk                       (_UINT32_(0xFFFFFFFF) << DSU_END_END_Pos)            /* (DSU_END) End Marker Mask */
#define DSU_END_END(value)                    (DSU_END_END_Msk & (_UINT32_(value) << DSU_END_END_Pos)) /* Assigment of value for END in the DSU_END register */
#define DSU_END_Msk                           _UINT32_(0xFFFFFFFF)                                 /* (DSU_END) Register Mask  */


/* -------- DSU_MEMTYPE : (DSU Offset: 0x1FCC) ( R/ 32) CoreSight ROM Table Memory Type -------- */
#define DSU_MEMTYPE_RESETVALUE                _UINT32_(0x00)                                       /*  (DSU_MEMTYPE) CoreSight ROM Table Memory Type  Reset Value */

#define DSU_MEMTYPE_SMEMP_Pos                 _UINT32_(0)                                          /* (DSU_MEMTYPE) System Memory Present Position */
#define DSU_MEMTYPE_SMEMP_Msk                 (_UINT32_(0x1) << DSU_MEMTYPE_SMEMP_Pos)             /* (DSU_MEMTYPE) System Memory Present Mask */
#define DSU_MEMTYPE_SMEMP(value)              (DSU_MEMTYPE_SMEMP_Msk & (_UINT32_(value) << DSU_MEMTYPE_SMEMP_Pos)) /* Assigment of value for SMEMP in the DSU_MEMTYPE register */
#define DSU_MEMTYPE_Msk                       _UINT32_(0x00000001)                                 /* (DSU_MEMTYPE) Register Mask  */


/* -------- DSU_PID4 : (DSU Offset: 0x1FD0) ( R/ 32) Peripheral Identification 4 -------- */
#define DSU_PID4_RESETVALUE                   _UINT32_(0x00)                                       /*  (DSU_PID4) Peripheral Identification 4  Reset Value */

#define DSU_PID4_JEPCC_Pos                    _UINT32_(0)                                          /* (DSU_PID4) JEP-106 Continuation Code Position */
#define DSU_PID4_JEPCC_Msk                    (_UINT32_(0xF) << DSU_PID4_JEPCC_Pos)                /* (DSU_PID4) JEP-106 Continuation Code Mask */
#define DSU_PID4_JEPCC(value)                 (DSU_PID4_JEPCC_Msk & (_UINT32_(value) << DSU_PID4_JEPCC_Pos)) /* Assigment of value for JEPCC in the DSU_PID4 register */
#define DSU_PID4_FKBC_Pos                     _UINT32_(4)                                          /* (DSU_PID4) 4KB Count Position */
#define DSU_PID4_FKBC_Msk                     (_UINT32_(0xF) << DSU_PID4_FKBC_Pos)                 /* (DSU_PID4) 4KB Count Mask */
#define DSU_PID4_FKBC(value)                  (DSU_PID4_FKBC_Msk & (_UINT32_(value) << DSU_PID4_FKBC_Pos)) /* Assigment of value for FKBC in the DSU_PID4 register */
#define DSU_PID4_Msk                          _UINT32_(0x000000FF)                                 /* (DSU_PID4) Register Mask  */


/* -------- DSU_PID0 : (DSU Offset: 0x1FE0) ( R/ 32) Peripheral Identification 0 -------- */
#define DSU_PID0_RESETVALUE                   _UINT32_(0xD0)                                       /*  (DSU_PID0) Peripheral Identification 0  Reset Value */

#define DSU_PID0_PARTNBL_Pos                  _UINT32_(0)                                          /* (DSU_PID0) Part Number Low Position */
#define DSU_PID0_PARTNBL_Msk                  (_UINT32_(0xFF) << DSU_PID0_PARTNBL_Pos)             /* (DSU_PID0) Part Number Low Mask */
#define DSU_PID0_PARTNBL(value)               (DSU_PID0_PARTNBL_Msk & (_UINT32_(value) << DSU_PID0_PARTNBL_Pos)) /* Assigment of value for PARTNBL in the DSU_PID0 register */
#define DSU_PID0_Msk                          _UINT32_(0x000000FF)                                 /* (DSU_PID0) Register Mask  */


/* -------- DSU_PID1 : (DSU Offset: 0x1FE4) ( R/ 32) Peripheral Identification 1 -------- */
#define DSU_PID1_RESETVALUE                   _UINT32_(0xFC)                                       /*  (DSU_PID1) Peripheral Identification 1  Reset Value */

#define DSU_PID1_PARTNBH_Pos                  _UINT32_(0)                                          /* (DSU_PID1) Part Number High Position */
#define DSU_PID1_PARTNBH_Msk                  (_UINT32_(0xF) << DSU_PID1_PARTNBH_Pos)              /* (DSU_PID1) Part Number High Mask */
#define DSU_PID1_PARTNBH(value)               (DSU_PID1_PARTNBH_Msk & (_UINT32_(value) << DSU_PID1_PARTNBH_Pos)) /* Assigment of value for PARTNBH in the DSU_PID1 register */
#define DSU_PID1_JEPIDCL_Pos                  _UINT32_(4)                                          /* (DSU_PID1) Low part of the JEP-106 Identity Code Position */
#define DSU_PID1_JEPIDCL_Msk                  (_UINT32_(0xF) << DSU_PID1_JEPIDCL_Pos)              /* (DSU_PID1) Low part of the JEP-106 Identity Code Mask */
#define DSU_PID1_JEPIDCL(value)               (DSU_PID1_JEPIDCL_Msk & (_UINT32_(value) << DSU_PID1_JEPIDCL_Pos)) /* Assigment of value for JEPIDCL in the DSU_PID1 register */
#define DSU_PID1_Msk                          _UINT32_(0x000000FF)                                 /* (DSU_PID1) Register Mask  */


/* -------- DSU_PID2 : (DSU Offset: 0x1FE8) ( R/ 32) Peripheral Identification 2 -------- */
#define DSU_PID2_RESETVALUE                   _UINT32_(0x09)                                       /*  (DSU_PID2) Peripheral Identification 2  Reset Value */

#define DSU_PID2_JEPIDCH_Pos                  _UINT32_(0)                                          /* (DSU_PID2) JEP-106 Identity Code High Position */
#define DSU_PID2_JEPIDCH_Msk                  (_UINT32_(0x7) << DSU_PID2_JEPIDCH_Pos)              /* (DSU_PID2) JEP-106 Identity Code High Mask */
#define DSU_PID2_JEPIDCH(value)               (DSU_PID2_JEPIDCH_Msk & (_UINT32_(value) << DSU_PID2_JEPIDCH_Pos)) /* Assigment of value for JEPIDCH in the DSU_PID2 register */
#define DSU_PID2_JEPU_Pos                     _UINT32_(3)                                          /* (DSU_PID2) JEP-106 Identity Code is used Position */
#define DSU_PID2_JEPU_Msk                     (_UINT32_(0x1) << DSU_PID2_JEPU_Pos)                 /* (DSU_PID2) JEP-106 Identity Code is used Mask */
#define DSU_PID2_JEPU(value)                  (DSU_PID2_JEPU_Msk & (_UINT32_(value) << DSU_PID2_JEPU_Pos)) /* Assigment of value for JEPU in the DSU_PID2 register */
#define DSU_PID2_REVISION_Pos                 _UINT32_(4)                                          /* (DSU_PID2) Revision Number Position */
#define DSU_PID2_REVISION_Msk                 (_UINT32_(0xF) << DSU_PID2_REVISION_Pos)             /* (DSU_PID2) Revision Number Mask */
#define DSU_PID2_REVISION(value)              (DSU_PID2_REVISION_Msk & (_UINT32_(value) << DSU_PID2_REVISION_Pos)) /* Assigment of value for REVISION in the DSU_PID2 register */
#define DSU_PID2_Msk                          _UINT32_(0x000000FF)                                 /* (DSU_PID2) Register Mask  */


/* -------- DSU_PID3 : (DSU Offset: 0x1FEC) ( R/ 32) Peripheral Identification 3 -------- */
#define DSU_PID3_RESETVALUE                   _UINT32_(0x00)                                       /*  (DSU_PID3) Peripheral Identification 3  Reset Value */

#define DSU_PID3_CUSMOD_Pos                   _UINT32_(0)                                          /* (DSU_PID3) ARM CUSMOD Position */
#define DSU_PID3_CUSMOD_Msk                   (_UINT32_(0xF) << DSU_PID3_CUSMOD_Pos)               /* (DSU_PID3) ARM CUSMOD Mask */
#define DSU_PID3_CUSMOD(value)                (DSU_PID3_CUSMOD_Msk & (_UINT32_(value) << DSU_PID3_CUSMOD_Pos)) /* Assigment of value for CUSMOD in the DSU_PID3 register */
#define DSU_PID3_REVAND_Pos                   _UINT32_(4)                                          /* (DSU_PID3) Revision Number Position */
#define DSU_PID3_REVAND_Msk                   (_UINT32_(0xF) << DSU_PID3_REVAND_Pos)               /* (DSU_PID3) Revision Number Mask */
#define DSU_PID3_REVAND(value)                (DSU_PID3_REVAND_Msk & (_UINT32_(value) << DSU_PID3_REVAND_Pos)) /* Assigment of value for REVAND in the DSU_PID3 register */
#define DSU_PID3_Msk                          _UINT32_(0x000000FF)                                 /* (DSU_PID3) Register Mask  */


/* -------- DSU_CID0 : (DSU Offset: 0x1FF0) ( R/ 32) Component Identification 0 -------- */
#define DSU_CID0_RESETVALUE                   _UINT32_(0x0D)                                       /*  (DSU_CID0) Component Identification 0  Reset Value */

#define DSU_CID0_PREAMBLEB0_Pos               _UINT32_(0)                                          /* (DSU_CID0) Preamble Byte 0 Position */
#define DSU_CID0_PREAMBLEB0_Msk               (_UINT32_(0xFF) << DSU_CID0_PREAMBLEB0_Pos)          /* (DSU_CID0) Preamble Byte 0 Mask */
#define DSU_CID0_PREAMBLEB0(value)            (DSU_CID0_PREAMBLEB0_Msk & (_UINT32_(value) << DSU_CID0_PREAMBLEB0_Pos)) /* Assigment of value for PREAMBLEB0 in the DSU_CID0 register */
#define DSU_CID0_Msk                          _UINT32_(0x000000FF)                                 /* (DSU_CID0) Register Mask  */


/* -------- DSU_CID1 : (DSU Offset: 0x1FF4) ( R/ 32) Component Identification 1 -------- */
#define DSU_CID1_RESETVALUE                   _UINT32_(0x10)                                       /*  (DSU_CID1) Component Identification 1  Reset Value */

#define DSU_CID1_PREAMBLE_Pos                 _UINT32_(0)                                          /* (DSU_CID1) Preamble Position */
#define DSU_CID1_PREAMBLE_Msk                 (_UINT32_(0xF) << DSU_CID1_PREAMBLE_Pos)             /* (DSU_CID1) Preamble Mask */
#define DSU_CID1_PREAMBLE(value)              (DSU_CID1_PREAMBLE_Msk & (_UINT32_(value) << DSU_CID1_PREAMBLE_Pos)) /* Assigment of value for PREAMBLE in the DSU_CID1 register */
#define DSU_CID1_CCLASS_Pos                   _UINT32_(4)                                          /* (DSU_CID1) Component Class Position */
#define DSU_CID1_CCLASS_Msk                   (_UINT32_(0xF) << DSU_CID1_CCLASS_Pos)               /* (DSU_CID1) Component Class Mask */
#define DSU_CID1_CCLASS(value)                (DSU_CID1_CCLASS_Msk & (_UINT32_(value) << DSU_CID1_CCLASS_Pos)) /* Assigment of value for CCLASS in the DSU_CID1 register */
#define DSU_CID1_Msk                          _UINT32_(0x000000FF)                                 /* (DSU_CID1) Register Mask  */


/* -------- DSU_CID2 : (DSU Offset: 0x1FF8) ( R/ 32) Component Identification 2 -------- */
#define DSU_CID2_RESETVALUE                   _UINT32_(0x05)                                       /*  (DSU_CID2) Component Identification 2  Reset Value */

#define DSU_CID2_PREAMBLEB2_Pos               _UINT32_(0)                                          /* (DSU_CID2) Preamble Byte 2 Position */
#define DSU_CID2_PREAMBLEB2_Msk               (_UINT32_(0xFF) << DSU_CID2_PREAMBLEB2_Pos)          /* (DSU_CID2) Preamble Byte 2 Mask */
#define DSU_CID2_PREAMBLEB2(value)            (DSU_CID2_PREAMBLEB2_Msk & (_UINT32_(value) << DSU_CID2_PREAMBLEB2_Pos)) /* Assigment of value for PREAMBLEB2 in the DSU_CID2 register */
#define DSU_CID2_Msk                          _UINT32_(0x000000FF)                                 /* (DSU_CID2) Register Mask  */


/* -------- DSU_CID3 : (DSU Offset: 0x1FFC) ( R/ 32) Component Identification 3 -------- */
#define DSU_CID3_RESETVALUE                   _UINT32_(0xB1)                                       /*  (DSU_CID3) Component Identification 3  Reset Value */

#define DSU_CID3_PREAMBLEB3_Pos               _UINT32_(0)                                          /* (DSU_CID3) Preamble Byte 3 Position */
#define DSU_CID3_PREAMBLEB3_Msk               (_UINT32_(0xFF) << DSU_CID3_PREAMBLEB3_Pos)          /* (DSU_CID3) Preamble Byte 3 Mask */
#define DSU_CID3_PREAMBLEB3(value)            (DSU_CID3_PREAMBLEB3_Msk & (_UINT32_(value) << DSU_CID3_PREAMBLEB3_Pos)) /* Assigment of value for PREAMBLEB3 in the DSU_CID3 register */
#define DSU_CID3_Msk                          _UINT32_(0x000000FF)                                 /* (DSU_CID3) Register Mask  */


/** \brief DSU register offsets definitions */
#define DSU_CTRL_REG_OFST              _UINT32_(0x00)      /* (DSU_CTRL) Control Offset */
#define DSU_STATUSA_REG_OFST           _UINT32_(0x01)      /* (DSU_STATUSA) Status A Offset */
#define DSU_STATUSB_REG_OFST           _UINT32_(0x02)      /* (DSU_STATUSB) Status B Offset */
#define DSU_ADDR_REG_OFST              _UINT32_(0x04)      /* (DSU_ADDR) Address Offset */
#define DSU_LENGTH_REG_OFST            _UINT32_(0x08)      /* (DSU_LENGTH) Length Offset */
#define DSU_DATA_REG_OFST              _UINT32_(0x0C)      /* (DSU_DATA) Data Offset */
#define DSU_DCC_REG_OFST               _UINT32_(0x10)      /* (DSU_DCC) Debug Communication Channel n Offset */
#define DSU_DCC0_REG_OFST              _UINT32_(0x10)      /* (DSU_DCC0) Debug Communication Channel n Offset */
#define DSU_DCC1_REG_OFST              _UINT32_(0x14)      /* (DSU_DCC1) Debug Communication Channel n Offset */
#define DSU_DID_REG_OFST               _UINT32_(0x18)      /* (DSU_DID) Device Identification Offset */
#define DSU_ENTRY0_REG_OFST            _UINT32_(0x1000)    /* (DSU_ENTRY0) CoreSight ROM Table Entry 0 Offset */
#define DSU_ENTRY1_REG_OFST            _UINT32_(0x1004)    /* (DSU_ENTRY1) CoreSight ROM Table Entry 1 Offset */
#define DSU_END_REG_OFST               _UINT32_(0x1008)    /* (DSU_END) CoreSight ROM Table End Offset */
#define DSU_MEMTYPE_REG_OFST           _UINT32_(0x1FCC)    /* (DSU_MEMTYPE) CoreSight ROM Table Memory Type Offset */
#define DSU_PID4_REG_OFST              _UINT32_(0x1FD0)    /* (DSU_PID4) Peripheral Identification 4 Offset */
#define DSU_PID0_REG_OFST              _UINT32_(0x1FE0)    /* (DSU_PID0) Peripheral Identification 0 Offset */
#define DSU_PID1_REG_OFST              _UINT32_(0x1FE4)    /* (DSU_PID1) Peripheral Identification 1 Offset */
#define DSU_PID2_REG_OFST              _UINT32_(0x1FE8)    /* (DSU_PID2) Peripheral Identification 2 Offset */
#define DSU_PID3_REG_OFST              _UINT32_(0x1FEC)    /* (DSU_PID3) Peripheral Identification 3 Offset */
#define DSU_CID0_REG_OFST              _UINT32_(0x1FF0)    /* (DSU_CID0) Component Identification 0 Offset */
#define DSU_CID1_REG_OFST              _UINT32_(0x1FF4)    /* (DSU_CID1) Component Identification 1 Offset */
#define DSU_CID2_REG_OFST              _UINT32_(0x1FF8)    /* (DSU_CID2) Component Identification 2 Offset */
#define DSU_CID3_REG_OFST              _UINT32_(0x1FFC)    /* (DSU_CID3) Component Identification 3 Offset */

#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
/** \brief DSU register API structure */
typedef struct
{  /* Device Service Unit */
  __O   uint8_t                        DSU_CTRL;           /**< Offset: 0x00 ( /W  8) Control */
  __IO  uint8_t                        DSU_STATUSA;        /**< Offset: 0x01 (R/W  8) Status A */
  __I   uint8_t                        DSU_STATUSB;        /**< Offset: 0x02 (R/   8) Status B */
  __I   uint8_t                        Reserved1[0x01];
  __IO  uint32_t                       DSU_ADDR;           /**< Offset: 0x04 (R/W  32) Address */
  __IO  uint32_t                       DSU_LENGTH;         /**< Offset: 0x08 (R/W  32) Length */
  __IO  uint32_t                       DSU_DATA;           /**< Offset: 0x0C (R/W  32) Data */
  __IO  uint32_t                       DSU_DCC[2];         /**< Offset: 0x10 (R/W  32) Debug Communication Channel n */
  __I   uint32_t                       DSU_DID;            /**< Offset: 0x18 (R/   32) Device Identification */
  __I   uint8_t                        Reserved2[0xFE4];
  __I   uint32_t                       DSU_ENTRY0;         /**< Offset: 0x1000 (R/   32) CoreSight ROM Table Entry 0 */
  __I   uint32_t                       DSU_ENTRY1;         /**< Offset: 0x1004 (R/   32) CoreSight ROM Table Entry 1 */
  __I   uint32_t                       DSU_END;            /**< Offset: 0x1008 (R/   32) CoreSight ROM Table End */
  __I   uint8_t                        Reserved3[0xFC0];
  __I   uint32_t                       DSU_MEMTYPE;        /**< Offset: 0x1FCC (R/   32) CoreSight ROM Table Memory Type */
  __I   uint32_t                       DSU_PID4;           /**< Offset: 0x1FD0 (R/   32) Peripheral Identification 4 */
  __I   uint8_t                        Reserved4[0x0C];
  __I   uint32_t                       DSU_PID0;           /**< Offset: 0x1FE0 (R/   32) Peripheral Identification 0 */
  __I   uint32_t                       DSU_PID1;           /**< Offset: 0x1FE4 (R/   32) Peripheral Identification 1 */
  __I   uint32_t                       DSU_PID2;           /**< Offset: 0x1FE8 (R/   32) Peripheral Identification 2 */
  __I   uint32_t                       DSU_PID3;           /**< Offset: 0x1FEC (R/   32) Peripheral Identification 3 */
  __I   uint32_t                       DSU_CID0;           /**< Offset: 0x1FF0 (R/   32) Component Identification 0 */
  __I   uint32_t                       DSU_CID1;           /**< Offset: 0x1FF4 (R/   32) Component Identification 1 */
  __I   uint32_t                       DSU_CID2;           /**< Offset: 0x1FF8 (R/   32) Component Identification 2 */
  __I   uint32_t                       DSU_CID3;           /**< Offset: 0x1FFC (R/   32) Component Identification 3 */
} dsu_registers_t;


#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
#endif /* _SAMR21_DSU_COMPONENT_H_ */
