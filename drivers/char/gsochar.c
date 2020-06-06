/**
 * @file   	gsochar.c
 * @author 	Deepak Khatri
 * @date   	6 June 2020
 * @version 0.1
 * @brief   Warmp up task for GSoC2020.
 * @see 	https://github.com/lorforlinux/gsoc-simple-char
 */

#include <linux/init.h>           // Macros used to mark up functions e.g. __init __exit
#include <linux/module.h>         // Core header for loading LKMs into the kernel
#include <linux/device.h>         // Header to support the kernel Driver Model
#include <linux/kernel.h>         // Contains types, macros, functions for the kernel
#include <linux/fs.h>             // Header for the Linux file system support
#include <linux/uaccess.h>        // Required for the copy to user function

#define  DEVICE_NAME "gsochar"    ///< The device will appear at /dev/gsochar using this value
#define  CLASS_NAME  "gsoc"       ///< The device class -- this is a character device driver
#define BUFFER_SIZE 1024

MODULE_LICENSE("GPL");            ///< The license type -- this affects available functionality
MODULE_AUTHOR("Deepak khatri");    ///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("A simple Linux char driver for the GSoc warmup task.");  ///< The description -- see modinfo
MODULE_VERSION("0.1");

static int    majorNumber;                  ///< Stores the device number -- determined automatically
static char   message[BUFFER_SIZE] = {0};   ///< Memory for the string that is passed from userspace
static int    numberOpens = 0;              ///< Counts the number of times the device is opened
static struct class*  gsocharClass  = NULL; ///< The device-driver class struct pointer
static struct device* gsocharDevice = NULL; ///< The device-driver device struct pointer
static int placeholder = 0;

/** @brief The device open function that is called each time the device is opened
 *  This will only increment the numberOpens counter in this case.
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_open(struct inode *inodep, struct file *filep){
   numberOpens++;
   printk(KERN_INFO "gsochar: Device has been opened %d time(s)\n", numberOpens);
   return 0;
}

/** @brief This function is called whenever device is being read from user space i.e. data is
 *  being sent from the device to the user. In this case is uses the copy_to_user() function to
 *  send the buffer string to the user and captures any errors.
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 *  @param buffer The pointer to the buffer to which this function writes the data
 *  @param len The length of the b
 *  @param offset The offset if required
 */

static ssize_t dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset){

	int bytesRead;
	int bytesToRead = BUFFER_SIZE - *offset;

	// If we are at the end of the file, STOP READING!
	if (bytesToRead == 0){
		printk(KERN_ALERT "gsochar: Reached the end of the file");
		return bytesToRead;
	}
	
	// Get bytes read by subtracting return of copy_to_user (returns unread bytes)
	bytesRead = bytesToRead - copy_to_user(buffer, message + *offset, bytesToRead);
	printk(KERN_ALERT "gsochar: READING with GSoC Simple Character Driver. Reading %d bytes\n", bytesRead);

	// Set offset so that we can eventually reach the end of the file
	*offset += bytesRead;

	return bytesRead;
}

/** @brief This function is called whenever the device is being written to from user space i.e.
 *  data is sent to the device from the user. The data is copied to the message[] array in this
 *  LKM using the sprintf() function along with the length of the string.
 *  @param filep A pointer to a file object
 *  @param buffer The buffer to that contains the string to write to the device
 *  @param len The length of the array of data that is being passed in the const char buffer
 *  @param offset The offset if required
 */

static ssize_t dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset){

	int bytesToWrite;
	int bytesWritten;
	int bytesAvailable = BUFFER_SIZE - *offset - placeholder;

	// Make sure there is sufficient space
	if(bytesAvailable > len){
		bytesToWrite = len; 
	}
	else{
		bytesToWrite = bytesAvailable;
	}

	//Get bites written by subtracting unwritten bites from retun of copy_from_user
	bytesWritten = bytesToWrite - copy_from_user(message + *offset + placeholder, buffer, bytesToWrite);
	
	// If no space left:
	if(bytesWritten == 0){
		printk(KERN_ALERT "gsochar: The device is out of space.\n");
	}
	else{
		//Increment offset and placeholder
		*offset += bytesWritten;
		placeholder += bytesWritten;

		printk(KERN_ALERT "gsochar: WRITING with Simple Character Driver. Writing %d bytes\n", bytesWritten);
	}
	return bytesWritten;
}

/** @brief The device release function that is called whenever the device is closed/released by
 *  the userspace program
 *  @param inodep A pointer to an inode object (defined in linux/fs.h)
 *  @param filep A pointer to a file object (defined in linux/fs.h)
 */
static int dev_release(struct inode *inodep, struct file *filep){
   printk(KERN_INFO "gsochar: Device successfully closed\n");
   return 0;
}

/** @brief Devices are represented as file structure in the kernel. The file_operations structure from
 *  /linux/fs.h lists the callback functions that you wish to associated with your file operations
 *  using a C99 syntax structure. char devices usually implement open, read, write and release calls
 */
static struct file_operations fops =
{
   .open = dev_open,
   .read = dev_read,
   .write = dev_write,
   .release = dev_release,
};

/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point.
 *  @return returns 0 if successful
 */
static int __init gsochar_init(void){
   printk(KERN_INFO "gsochar: Initializing the gsochar LKM\n");

   // Try to dynamically allocate a major number for the device -- more difficult but worth it
   majorNumber = register_chrdev(0, DEVICE_NAME, &fops);
   if (majorNumber<0){
      printk(KERN_ALERT "gsochar failed to register a major number\n");
      return majorNumber;
   }
   printk(KERN_INFO "gsochar: registered correctly with major number %d\n", majorNumber);

   // Register the device class
   gsocharClass = class_create(THIS_MODULE, CLASS_NAME);
   if (IS_ERR(gsocharClass)){                // Check for error and clean up if there is
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_ALERT "Failed to register device class\n");
      return PTR_ERR(gsocharClass);          // Correct way to return an error on a pointer
   }
   printk(KERN_INFO "gsochar: device class registered correctly\n");

   // Register the device driver
   gsocharDevice = device_create(gsocharClass, NULL, MKDEV(majorNumber, 0), NULL, DEVICE_NAME);
   if (IS_ERR(gsocharDevice)){               // Clean up if there is an error
      class_destroy(gsocharClass);           // Repeated code but the alternative is goto statements
      unregister_chrdev(majorNumber, DEVICE_NAME);
      printk(KERN_ALERT "Failed to create the device\n");
      return PTR_ERR(gsocharDevice);
   }
   printk(KERN_INFO "gsochar: device class created correctly\n"); // Made it! device was initialized
   return 0;
}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit gsochar_exit(void){
   device_destroy(gsocharClass, MKDEV(majorNumber, 0));     // remove the device
   class_unregister(gsocharClass);                          // unregister the device class
   class_destroy(gsocharClass);                             // remove the device class
   unregister_chrdev(majorNumber, DEVICE_NAME);             // unregister the major number
   printk(KERN_INFO "gsochar: Goodbye from the LKM!\n");
}

/** @brief A module must use the module_init() module_exit() macros from linux/init.h, which
 *  identify the initialization function at insertion time and the cleanup function (as
 *  listed above)
 */
module_init(gsochar_init);
module_exit(gsochar_exit);