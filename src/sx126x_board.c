#include "radio.h"
#include "sx126x.h"
#include "sx126x_board.h"

#include "compact.h"
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"

#include "compact.h"

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(0);

void SX126xIoInit
	( uint8_t spi_nss,uint8_t spi_miso, uint8_t spi_mosi, uint8_t spi_sck, 
	uint8_t busy, uint8_t rxen, uint8_t nrst, uint8_t dio1 )
{
	SX126x.BUSY = busy;
	SX126x.RXEN = rxen;
	SX126x.Reset = nrst;
	SX126x.DIO1 = dio1;
	SX126x.NSS = spi_nss;
	
	nrf_gpio_cfg_input(SX126x.BUSY, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_output(SX126x.RXEN);
	nrf_gpio_cfg_output(SX126x.Reset);
	nrf_gpio_cfg_output(SX126x.NSS);
	
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.frequency	  = NRF_DRV_SPI_FREQ_8M;
	spi_config.miso_pin	   = spi_miso;
	spi_config.mosi_pin	   = spi_mosi;
	spi_config.sck_pin		= spi_sck;
	APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, NULL, NULL));
	SX126x.Spi = spi;
}

void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
	if (!nrfx_gpiote_is_init())
		nrfx_gpiote_init();
	nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
	APP_ERROR_CHECK(nrfx_gpiote_in_init(SX126x.DIO1, &in_config, dioIrq));
	BoardDisableIrq( );
}

void SX126xIoDeInit( void ) { }

void SX126xReset( void )
{
	DelayMs( 10 );
	nrf_gpio_pin_write( SX126x.Reset, 0 );
	DelayMs( 20 );
	nrf_gpio_pin_write( SX126x.Reset, 1 );
	DelayMs( 10 );
}


void SX126xWakeup( void )
{

	nrf_gpio_pin_write(SX126x.NSS, 0 );
	
	uint8_t cmd[] = { RADIO_GET_STATUS, 0x0 };
	nrf_drv_spi_transfer( &SX126x.Spi, cmd, sizeof(cmd), NULL, 0 );

	nrf_gpio_pin_write(SX126x.NSS, 1 );

	// Wait for chip to be ready.
	SX126xWaitOnBusy( );

}

void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
	SX126xCheckDeviceReady( );

	nrf_gpio_pin_write( SX126x.NSS, 0 );

	nrf_drv_spi_transfer( &SX126x.Spi, (uint8_t *)&command, 1, NULL, 0 );

	nrf_drv_spi_transfer( &SX126x.Spi, buffer, size, NULL, 0 );

	nrf_gpio_pin_write( SX126x.NSS, 1 );

	if( command != RADIO_SET_SLEEP )
	{
		SX126xWaitOnBusy( );
	}
}

void SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
	SX126xCheckDeviceReady( );

	nrf_gpio_pin_write( SX126x.NSS, 0 );
	
	uint8_t cmd[] = { command, 0 };
	
	nrf_drv_spi_transfer( &SX126x.Spi, cmd, sizeof(cmd), NULL, 0 );

	nrf_drv_spi_transfer(&SX126x.Spi, NULL, 0, buffer, size);

	nrf_gpio_pin_write( SX126x.NSS, 1 );

	SX126xWaitOnBusy( );
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
	SX126xCheckDeviceReady( );

	nrf_gpio_pin_write( SX126x.NSS, 0 );
	
	uint8_t cmd[] = { RADIO_WRITE_REGISTER, ( address & 0xFF00 ) >> 8, address & 0x00FF };
	
	nrf_drv_spi_transfer(&SX126x.Spi, cmd, sizeof(cmd), NULL, 0);

	nrf_drv_spi_transfer(&SX126x.Spi, buffer, size, NULL, 0);

	nrf_gpio_pin_write( SX126x.NSS, 1 );

	SX126xWaitOnBusy( );
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
	SX126xWriteRegisters( address, &value, 1 );
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
	SX126xCheckDeviceReady( );

	nrf_gpio_pin_write( SX126x.NSS, 0 );
	
	uint8_t cmd[] = { RADIO_READ_REGISTER, ( address & 0xFF00 ) >> 8, address & 0x00FF, 0x0 };
	nrf_drv_spi_transfer(&SX126x.Spi, cmd, sizeof(cmd), NULL, 0);

	nrf_drv_spi_transfer(&SX126x.Spi, NULL, 0, buffer, size);

	nrf_gpio_pin_write( SX126x.NSS, 1 );
	
	SX126xWaitOnBusy( );
}

uint8_t SX126xReadRegister( uint16_t address )
{
	uint8_t data;
	SX126xReadRegisters( address, &data, 1 );
	return data;
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
	SX126xCheckDeviceReady( );

	nrf_gpio_pin_write( SX126x.NSS, 0 );
	
	uint8_t cmd[] = {RADIO_WRITE_BUFFER, offset};
	nrf_drv_spi_transfer(&SX126x.Spi, cmd, sizeof(cmd), NULL, 0);

	nrf_drv_spi_transfer(&SX126x.Spi, buffer, size, NULL ,0 );
	
	nrf_gpio_pin_write( SX126x.NSS, 1 );
	SX126xWaitOnBusy( );
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
	SX126xCheckDeviceReady( );

	nrf_gpio_pin_write( SX126x.NSS, 0 );
	
	uint8_t cmd[] = {RADIO_READ_BUFFER, offset, 0x0};
	nrf_drv_spi_transfer(&SX126x.Spi, cmd, sizeof(cmd), NULL, 0);

	nrf_drv_spi_transfer(&SX126x.Spi, NULL, 0, buffer, size);
		
	nrf_gpio_pin_write( SX126x.NSS, 1 );

	SX126xWaitOnBusy( );
}

void SX126xSetRfTxPower( int8_t power )
{
	SX126xSetTxParams( power, RADIO_RAMP_40_US );
}

bool SX126xCheckRfFrequency( uint32_t frequency )
{
	// Implement check. Currently all frequencies are supported
	return true;
}
