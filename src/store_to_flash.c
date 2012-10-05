/*
midopple: The code is FLASHD_Write(lastPageAddress, pBuffer, AT91C_IFLASH_PAGE_SIZE);
[19:14]	jebba	kthx tell karandex I answered your youtube Qs, but forgot to do it as a reply, so it is there but you may not have got an email.
[19:14]	kthx	jebba: I'll pass that on when karandex is around.
[19:15]	Kliment	midopple: lastpageaddress is the target address, pbuffer is the pointer to the data you want to write, the last parameter is the length of the data
[19:15]	Kliment	midopple: And to read it you just read from the pointer pointing to the page address
[19:17]	Kliment	midopple: The library code that implements it is http://pastebin.com/51hbHmGC
[19:17]	Kliment	midopple: I'll put it in at91lib for you to play with
[19:17]	Kliment	midopple: I recommend we write to the last page of the first bank of the flash
[19:18]	Kliment	midopple: This way we can do dual boot
[19:18]	Kliment	midopple: And run a bootloader off the flash

*/




