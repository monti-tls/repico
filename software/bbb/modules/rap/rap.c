#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexandre Monti");
MODULE_DESCRIPTION("");
MODULE_VERSION("");

static int __init rap_init()
{

}

static void __exit rap_exit()
{

}

module_init(rap_init);
module_exit(rap_exit);
