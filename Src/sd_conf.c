#include "sd_conf.h"

FRESULT sd_append (
    FIL* fp,            /* [OUT] File object to create */
    const char* path    /* [IN]  File name to be opened */
)
{
    FRESULT fr;

    /* Opens an existing file. If not exist, creates a new file. */
    fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (fr == FR_OK) {
        /* Seek to end of the file to append data */
        fr = f_lseek(fp, f_size(fp));
        if (fr != FR_OK)
            f_close(fp);
    }
    return fr;
}

/*
void sd_write(uint8_t* string, int length){
		// Mount SD Card
	  if(f_mount(&fs, "", 0) != FR_OK)
		Error_Handler();

	  // Open file to write
	  if(f_open(&fil, "test.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK)
		Error_Handler();

	  // Check free space
	  if(f_getfree("", &fre_clust, &pfs) != FR_OK)
		Error_Handler();

	  total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	  freespace = (uint32_t)(fre_clust * pfs->csize * 0.5);

	  // Free space is less than 1kb
	  if(freespace < 1)
		Error_Handler();

	  // Writing text
	  f_puts("Function is working properly!\n", &fil);

	  // Close file
	  if(f_close(&fil) != FR_OK)
		Error_Handler();

	  // Open file to read
	  if(f_open(&fil, "first.txt", FA_READ) != FR_OK)
		Error_Handler();

	  while(f_gets(buffer, sizeof(buffer), &fil))
	  {
		printf("%s", buffer);
	  }

	  // Close file
	  if(f_close(&fil) != FR_OK)
		Error_Handler();


	  // Unmount SDCARD
	  if(f_mount(NULL, "", 1) != FR_OK)
		Error_Handler();
}
*/
