/**
 * @file vmaccess.h
 * @author Prof. Dr. Wolfgang Fohl, HAW Hamburg
 * @date 2010
 * @brief This module defines function to read from and write to
 * virtual memory.
 */

#ifndef VMACCESS_H
#define VMACCESS_H

/**
 *****************************************************************************************
 *  @brief      This function reads an one byte from virtual memory.
 *              If this functions access virtual memory for the first time, the 
 *              virtual memory will be setup and initialized.
 *
 *  @param      address The virtual memory address the byte should be read from.
 * 
 *  @return     The byte read from virtual memory.
 ****************************************************************************************/
unsigned char vmem_read(int address);

/**
 *****************************************************************************************
 *  @brief      This function writes a byte to virtual memory.
 *              If this functions access virtual memory for the first time, the 
 *              virtual memory will be setup and initialized.
 *
 *  @param      address The virtual memory address the byte should be written to.
 *
 *  @param      data The byte value that should be written to virtual memory.
 * 
 *  @return     void
 ****************************************************************************************/
void vmem_write(int address, unsigned char data);

#endif
