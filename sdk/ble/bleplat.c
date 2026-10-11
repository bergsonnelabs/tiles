/**
 * bleplat.c -- BLE Platform Adaptation Layer (from ST working project)
 *
 * Provides BLEPLAT_* functions that the BLE stack library calls.
 * Delegates to BAES/BPKA/NVM stubs, BLE_TIMER, and HW_RNG.
 */

#include <stdint.h>
#include <string.h>
#include "app_common.h"
#include "bleplat.h"
#include "ble_timer.h"
#include "blestack.h"
#include "host_stack_if.h"

/* Forward declarations for stubs in sdk/wpan/wpan_sys.c */
extern void BAES_Reset(void);
extern void BPKA_Reset(void);
extern void BAES_EcbCrypt(const uint8_t *key, const uint8_t *input, uint8_t *output, int encrypt);
extern void BAES_CmacSetKey(const uint8_t *key);
extern void BAES_CmacCompute(const uint8_t *input, uint32_t input_length, uint8_t *output_tag);
extern int  BAES_CcmCrypt(uint8_t mode, const uint8_t *key, uint8_t iv_length, const uint8_t *iv,
                           uint16_t add_length, const uint8_t *add, uint32_t input_length,
                           const uint8_t *input, uint8_t tag_length, uint8_t *tag, uint8_t *output);
extern int  BPKA_StartP256Key(const uint32_t *local_private_key);
extern void BPKA_ReadP256Key(uint32_t *local_public_key);
extern int  BPKA_StartDhKey(const uint32_t *local_private_key, const uint32_t *remote_public_key);
extern int  BPKA_ReadDhKey(uint32_t *dh_key);
extern int  NVM_Add(uint8_t type, const uint8_t *data, uint16_t size,
                    const uint8_t *extra_data, uint16_t extra_size);
extern int  NVM_Get(int mode, uint8_t type, uint16_t offset, uint8_t *data, uint16_t size);
extern int  NVM_Compare(uint16_t offset, const uint8_t *data, uint16_t size);
extern void NVM_Discard(int mode);
extern void HW_RNG_Get(uint8_t n, uint32_t *val);

/*****************************************************************************/

void BLEPLAT_Init( void )
{
  BAES_Reset( );
  BPKA_Reset( );
  BLE_TIMER_Init();
}

/*****************************************************************************/

int BLEPLAT_NvmAdd( uint8_t type,
                    const uint8_t* data,
                    uint16_t size,
                    const uint8_t* extra_data,
                    uint16_t extra_size )
{
  return NVM_Add( type, data, size, extra_data, extra_size );
}

/*****************************************************************************/

int BLEPLAT_NvmGet( uint8_t mode,
                    uint8_t type,
                    uint16_t offset,
                    uint8_t* data,
                    uint16_t size )
{
  return NVM_Get( mode, type, offset, data, size );
}

/*****************************************************************************/

int BLEPLAT_NvmCompare( uint16_t offset,
                        const uint8_t* data,
                        uint16_t size )
{
  return NVM_Compare( offset, data, size );
}

/*****************************************************************************/

void BLEPLAT_NvmDiscard( uint8_t mode )
{
  NVM_Discard( mode );
}

/*****************************************************************************/

void BLEPLAT_RngGet( uint8_t n,
                     uint32_t* val )
{
  HW_RNG_Get( n, val );
}

/*****************************************************************************/

void BLEPLAT_AesEcbEncrypt( const uint8_t* key,
                            const uint8_t* input,
                            uint8_t* output )
{
  BAES_EcbCrypt( key, input, output, 1 );
}

/*****************************************************************************/

void BLEPLAT_AesCmacSetKey( const uint8_t* key )
{
  BAES_CmacSetKey( key );
}

/*****************************************************************************/

void BLEPLAT_AesCmacCompute( const uint8_t* input,
                             uint32_t input_length,
                             uint8_t* output_tag )
{
  BAES_CmacCompute( input, input_length, output_tag );
}

/*****************************************************************************/

int BLEPLAT_AesCcmCrypt( uint8_t mode,
                         const uint8_t* key,
                         uint8_t iv_length,
                         const uint8_t* iv,
                         uint16_t add_length,
                         const uint8_t* add,
                         uint32_t input_length,
                         const uint8_t* input,
                         uint8_t tag_length,
                         uint8_t* tag,
                         uint8_t* output )
{
  return BAES_CcmCrypt( mode, key, iv_length, iv, add_length, add,
                        input_length, input, tag_length, tag, output );
}

/*****************************************************************************/

int BLEPLAT_PkaStartP256Key( const uint32_t* local_private_key )
{
  return BPKA_StartP256Key( local_private_key );
}

/*****************************************************************************/

void BLEPLAT_PkaReadP256Key( uint32_t* local_public_key )
{
  BPKA_ReadP256Key( local_public_key );
}

/*****************************************************************************/

int BLEPLAT_PkaStartDhKey( const uint32_t* local_private_key,
                           const uint32_t* remote_public_key )
{
  return BPKA_StartDhKey( local_private_key, remote_public_key );
}

/*****************************************************************************/

int BLEPLAT_PkaReadDhKey( uint32_t* dh_key )
{
  return BPKA_ReadDhKey( dh_key );
}

/*****************************************************************************/

void BPKACB_Complete( void )
{
  BLEPLATCB_PkaComplete( );
  BleStackCB_Process( );
}

/*****************************************************************************/

uint8_t BLEPLAT_TimerStart( uint16_t id,
                            uint32_t timeout )
{
  return BLE_TIMER_Start( id, timeout );
}

/*****************************************************************************/

void BLEPLAT_TimerStop( uint16_t id )
{
  BLE_TIMER_Stop( id );
}

/*****************************************************************************/
