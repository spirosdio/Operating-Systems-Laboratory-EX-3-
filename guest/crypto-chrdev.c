/*
 * crypto-chrdev.c
 *
 * Implementation of character devices
 * for virtio-cryptodev device
 *
 * Vangelis Koukis <vkoukis@cslab.ece.ntua.gr>
 * Dimitris Siakavaras <jimsiak@cslab.ece.ntua.gr>
 * Stefanos Gerangelos <sgerag@cslab.ece.ntua.gr>
 *
 */
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>

#include "crypto.h"
#include "crypto-chrdev.h"
#include "debug.h"

#include "cryptodev.h"

/*
 * Global data
 */
struct cdev crypto_chrdev_cdev;

/**
 * Given the minor number of the inode return the crypto device
 * that owns that number.
 **/
static struct crypto_device *get_crypto_dev_by_minor(unsigned int minor)
{
	struct crypto_device *crdev;
	unsigned long flags;

	debug("Entering");

	spin_lock_irqsave(&crdrvdata.lock, flags);
	list_for_each_entry(crdev, &crdrvdata.devs, list)
	{
		if (crdev->minor == minor)
			goto out;
	}
	crdev = NULL;

out:
	spin_unlock_irqrestore(&crdrvdata.lock, flags);

	debug("Leaving");
	return crdev;
}

/*************************************
 * Implementation of file operations
 * for the Crypto character device
 *************************************/

static int crypto_chrdev_open(struct inode *inode, struct file *filp)
{
	/*
	1)Make sure to GIVE the right fd -> DONE
	2)Error check the entire function ->
	*/
	int ret = 0;
	int err; // find out its use??
	unsigned int len;
	struct crypto_open_file *crof; // It contains info for:1)Crypto Device 2)fd of open file in backend
	struct crypto_device *crdev;   // Crypto device -> meaning all the info about the requested ENC-DEC etc
	unsigned int *syscall_type;	   // type of action(syscall) we ask for host to take
	int *host_fd;
	debug("Entering");

	struct scatterlist syscall_type_sg, host_fd_sg;
	struct scatterlist *sgs[2]; // Buffer we send across the 'gap'

	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto fail;

	/* Associate this open file with the relevant crypto device. */
	crdev = get_crypto_dev_by_minor(iminor(inode)); // look again
	if (!crdev)
	{
		debug("Could not find crypto device with %u minor",
			  iminor(inode));
		ret = -ENODEV;
		goto fail;
	}
	// we put the above after nonseekable_open
	syscall_type = kzalloc(sizeof(*syscall_type), GFP_KERNEL); // Initialize "zero" memory for our pointer to call_type
	*syscall_type = VIRTIO_CRYPTODEV_SYSCALL_OPEN;

	host_fd = kzalloc(sizeof(*host_fd), GFP_KERNEL); // Initialize "zero" memory for our pointer to host_fd
	*host_fd = -1;

	crof = kzalloc(sizeof(*crof), GFP_KERNEL);
	if (!crof)
	{
		ret = -ENOMEM;
		goto fail;
	}
	crof->crdev = crdev; // initialize crof(crypto_open_file struct)
	crof->host_fd = -1;
	filp->private_data = crof; // save data of crof to private_data of FILE STRUCT
	struct virtqueue *vq = crdev->vq;

	/**
	 * We need two sg lists, one for syscall_type and one to get the
	 * file descriptor from the host. this is important i think we get it more know
	 **/
	/* ?? */
	int num_out = 0;
	int num_in = 0;

	sg_init_one(&syscall_type_sg, syscall_type, sizeof(*syscall_type));
	sgs[num_out++] = &syscall_type_sg;

	sg_init_one(&host_fd_sg, host_fd, sizeof(*host_fd));
	sgs[num_out + num_in++] = &host_fd_sg;

	/* do nothing */;
	/**
	 * Wait for the host to process our data.
	 *reverce - kick
	 **/
	/* ?? */
	spin_lock(&crdev->lock);
	err = virtqueue_add_sgs(vq, sgs, num_out, num_in, // we expose buffers to the other end (read and write buffers)
							&syscall_type_sg, GFP_ATOMIC);
	if (err < 0)
	{
		debug("virtqueue_add_sgs problem");
	}
	spin_unlock(&crdev->lock);
	virtqueue_kick(vq);
	while (virtqueue_get_buf(vq, &len) == NULL) {}

		// kati me spinlock

		/* If host failed to open() return -ENODEV.
			get buf
		*/
		/* ?? */
	crof->host_fd = *host_fd; // test here
	
	
	if (*host_fd < 0)
	{
		return -ENODEV;
	}
	kfree(syscall_type); // it should be freed or we will leave it with no purpose whatsoever in memory
	return *host_fd;	 // we return to the guest syscall open the SAME fd as the open of the host!(bazinga)
fail:
	debug("Leaving");
	return ret;
}

static int crypto_chrdev_release(struct inode *inode, struct file *filp)
{
	/*
	1)Make sure we GET the right fd -> DONE
	2)Error check the entire function ->
	*/
	long ret = 0;
	unsigned int len;

	int err;
	struct crypto_open_file *crof = filp->private_data; // we aquire from FILE struct the data given to us by open(inlcuding the fd)
	struct crypto_device *crdev = crof->crdev;			// we alose need the same crypto device as open!??
	unsigned int *syscall_type, *fd_type, *return_it;

	struct virtqueue *vq = crdev->vq;
	struct scatterlist syscall_type_sg, fd_type_sg, return_it_sg, *sgs[3];

	debug("Entering");

	int num_out = 0;
	int num_in = 0;

	syscall_type = kzalloc(sizeof(*syscall_type), GFP_KERNEL);
	*syscall_type = VIRTIO_CRYPTODEV_SYSCALL_CLOSE;

	return_it = kzalloc(sizeof(*return_it), GFP_KERNEL);
	*return_it = 0;

	fd_type = kzalloc(sizeof(*fd_type), GFP_KERNEL);
	*fd_type = crof->host_fd; // get fd from crof struct, which is connected to flip->private_data

	/**
	 * Send data to the host.
	 **/
	/* ?? */
	sg_init_one(&syscall_type_sg, syscall_type, sizeof(*syscall_type));
	sgs[num_out++] = &syscall_type_sg;

	sg_init_one(&fd_type_sg, fd_type, sizeof(*fd_type));
	sgs[num_out++] = &fd_type_sg;

	sg_init_one(&return_it_sg, return_it, sizeof(*return_it));
	sgs[num_out + num_in++] = &return_it_sg;

	/**
	 * Wait for the host to process our data.
	 **/
	/* ?? */
	// spin lock or unlock (&chrdev->lock)
	spin_lock(&crdev->lock);
	/* ?? Lock ?? */
	err = virtqueue_add_sgs(vq, sgs, num_out, num_in,
							&syscall_type_sg, GFP_ATOMIC);
	if (err < 0)
	{
		debug("virtqueue_add_sgs problem");
	}
	spin_unlock(&crdev->lock);
	virtqueue_kick(vq);
	while (virtqueue_get_buf(vq, &len) == NULL) /// get what he returned
		/* do nothing */;

	ret = *return_it;
	kfree(fd_type);
	kfree(syscall_type);
	// should we deallocate memory from flip->private_data? or is it deallocated by kfree(crof)???
	kfree(filp->private_data); // if pointer is NULL nothing happens so we are good
	kfree(crof);
	debug("Leaving");

	return ret;
}

static long crypto_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/*
	1)Take host_fd from open file -> DONE
	2)Take second argument so we check what cmd we have -> DONE
	3)Analyze the third argument, find out the best way to do this ->
	4)Error check the entire function ->

	Problems:
	1)We might have problem with NULL buffers in sgs
	2)
	*/
	long ret = 0;
	int err, *fd;
	struct crypto_open_file *crof = filp->private_data;
	struct crypto_device *crdev = crof->crdev;
	struct virtqueue *vq = crdev->vq;
	struct scatterlist syscall_type_sg, cmd2_sg, fd_sg, return_it_sg, *sgs[11];
	unsigned int num_out, num_in, len, *syscall_type, *cmd2, *return_it;

	debug("Entering");
	/**
	 * Allocate all data that will be sent to the host.
	 **/

	cmd2 = kzalloc(sizeof(*cmd2), GFP_KERNEL);
	*cmd2 = cmd;

	syscall_type = kzalloc(sizeof(*syscall_type), GFP_KERNEL);
	*syscall_type = VIRTIO_CRYPTODEV_SYSCALL_IOCTL;

	fd = kzalloc(sizeof(*fd), GFP_KERNEL);
	*fd = crof->host_fd;

	return_it = kzalloc(sizeof(*return_it), GFP_KERNEL);
	*return_it = 0;

	/**
	 *  These are common to all ioctl commands.
	 **/

	sg_init_one(&syscall_type_sg, syscall_type, sizeof(*syscall_type)); // out[0] -> we give syscall_type={OPEN,CLOSE,IOCTL}
	sgs[0] = &syscall_type_sg;

	sg_init_one(&fd_sg, fd, sizeof(*fd)); // out[1] -> we give file descriptor
	sgs[1] = &fd_sg;

	sg_init_one(&cmd2_sg, cmd2, sizeof(*cmd2)); // out[2] -> we give cmd name={CIOCGSESSION,CIOCFSESSION,CIOCCRYPT}
	sgs[2] = &cmd2_sg;

	sg_init_one(&return_it_sg, return_it, sizeof(*return_it)); // in[0] -> we aquire through host the returned value of ioctl()
	sgs[9] = &return_it_sg;

	/* ?? */
	/**
	 *  Add all the cmd2 specific sg lists.
	 **/

	// Initialize structs for ENC-DEC
	struct session_op *sess1;
	struct crypt_op *cryp1;

	sess1 = kzalloc(sizeof(*sess1), GFP_KERNEL);

	cryp1 = kzalloc(sizeof(*cryp1), GFP_KERNEL);

////////////111111111111111111

		// Initializations of parameters
		__u32 *ses;
		__u8 *key;


///////////////////////222222222222222222222222222222222222
	// Initializations of parameters
		__u32 *ses1;
		__u8 *key2;
		__u32 *sess_id;


		////////////////33333333333333333333333333333
		__u8 *src;
		__u8 *iv;
		__u16 *flags;
		__u8 *dst;
		__u16 *op;
		__u32 *len_s;



	switch (*cmd2)
	{
	case CIOCGSESSION:
		debug("CIOCGSESSION");
		copy_from_user(sess1, (const void __user *)arg, sizeof(sess1)); // check again validity of (const void __user*)
		/*
		sess.cipher = CRYPTO_AES_CBC;
		sess.keylen = KEY_SIZE;
		sess.key = (__u8  __user *)data.key;
		*/

		key = kzalloc(sizeof(*key), GFP_KERNEL);
		*key = sess1->key;

		ses = kzalloc(sizeof(*ses), GFP_KERNEL);

		// Transfer data related to struct
		struct scatterlist key_sg, ses_sg;
		sg_init_one(&key_sg, key, sizeof(*key));
		sgs[3] = &key_sg; // out[3] -> we give 'key' for CIOCGSESSION

		sg_init_one(&ses_sg, ses, sizeof(*ses));
		sgs[10] = &ses_sg; // in[1] -> we aquire ses id from CIOCGSESSION

		break;

	case CIOCFSESSION:
		debug("CIOCFSESSION");
		copy_from_user(sess1, (const void __user *)arg, sizeof(sess1)); // check again validity of (const void __user*)
		/*
		sess.cipher = CRYPTO_AES_CBC;
		sess.keylen = KEY_SIZE;
		sess.key = (__u8  __user *)data.key;
		*/

	

		ses1 = kzalloc(sizeof(*ses1), GFP_KERNEL);
		*ses1 = sess1->ses;

		key2 = kzalloc(sizeof(*key2), GFP_KERNEL);
		*key2 = sess1->key;

		// Tranfer data related to struct
		struct scatterlist ses_sg1, key_sg1;
		sg_init_one(&ses_sg1, ses1, sizeof(*ses1));
		sgs[3] = &ses_sg1; // out[3] -> we give 'key' for CIOCFSESSION

		sg_init_one(&key_sg1, key2, sizeof(*key2));
		sgs[4] = &key_sg1; // out[4] -> we give 'key' for CIOCFSESSION

		break;

	case CIOCCRYPT:
		debug("CIOCCRYPT");
		copy_from_user(cryp1, (const void __user *)arg, sizeof(cryp1)); // check again validity of (const void __user*)

		/*
		struct crypt_op {
	__u32	ses;		// session identifier
		__u16	op;		// COP_ENCRYPT or COP_DECRYPT
		__u16	flags;		// see COP_FLAG_*
		__u32	len;		// length of source data
		__u8	__user* src;	// source data
		__u8	__user* dst;	// pointer to output data
		// pointer to output data for hash/MAC operations
		__u8	__user* mac;
		// initialization vector for encryption operations
		__u8	__user* iv;
	};
		*/

		/*
		//cryp.ses = sess.ses;								//using pointer and we are ready -> we only need to transfer data
		//cryp.len = sizeof(data.in);							//using pointer and we are ready -> we only need to transfer data
		//cryp.src = (__u8 __user *)data.in;					//copy_from_user and we are ready -> we only need to transfer data
		//cryp.dst = (__u8 __user *)data.encrypted;			//in[1]
		//cryp.iv = (__u8 __user *)data.iv;					//copy_from_user and we are ready -> we only need to transfer data
		cryp.op = COP_ENCRYPT;								//using pointer and we are ready -> we only need to transfer data
		*/

		// Initializations of parameters
		sess_id = kzalloc(sizeof(*sess_id), GFP_KERNEL);
		*sess_id = cryp1->ses;

		len_s = kzalloc(sizeof(*len_s), GFP_KERNEL);
		*len_s = cryp1->len;

		op = kzalloc(sizeof(*op), GFP_KERNEL);
		*op = cryp1->op;

		src = kzalloc(sizeof(*src), GFP_KERNEL);
		copy_from_user(&src, (const void __user *)cryp1->src, sizeof(src));

		dst = kzalloc(sizeof(*dst), GFP_KERNEL);

		flags = kzalloc(sizeof(*flags), GFP_KERNEL);
		*flags = cryp1->flags;

		iv = kzalloc(sizeof(*iv), GFP_KERNEL);
		copy_from_user(&iv, (const void __user *)cryp1->iv, sizeof(iv));

		// Transfer data related to struct
		struct scatterlist sess_id_sg, len_s_sg, op_sg, src_sg, dst_sg, iv_sg, flags_sg;

		sg_init_one(&op_sg, op, sizeof(*op));
		sgs[3] = &op_sg;

		sg_init_one(&src_sg, src, sizeof(*src));
		sgs[4] = &src_sg;

		sg_init_one(&sess_id_sg, sess_id, sizeof(*sess_id));
		sgs[5] = &sess_id_sg;

		sg_init_one(&iv_sg, iv, sizeof(*iv));
		sgs[6] = &iv_sg;

		sg_init_one(&len_s_sg, len_s, sizeof(*len_s));
		sgs[7] = &len_s_sg;

		sg_init_one(&flags_sg, flags, sizeof(*flags));
		sgs[8] = &flags_sg;

		sg_init_one(&dst_sg, dst, sizeof(*dst));
		sgs[10] = &dst_sg;

		break;

	default:
		debug("Unsupported ioctl command");

		break;
	}

	/**
	 * Wait for the host to process our data.
	????????????????????????what we do here??????????????????????????????????
	 **/
	/* ?? */

	// spin lock (&chrdev->lock)
	spin_lock(&crdev->lock);
	/* ?? Lock ?? */
	err = virtqueue_add_sgs(vq, sgs, 9, 2, &syscall_type_sg, GFP_ATOMIC);
	if (err < 0)
	{
		debug("virtqueue_add_sgs problem");
	}
	spin_unlock(&crdev->lock);
	virtqueue_kick(vq);
	while (virtqueue_get_buf(vq, &len) == NULL) /// get what he returned
		/* do nothing */;

	// return and copy_to_user stuff
	ret = *return_it;

	if (*cmd2 == CIOCGSESSION)
	{
		int i = 0;
		i = copy_to_user((const void __user *)arg, sess1, sizeof(sess1));
		if (i != 0)
		{
			debug("error with copy_to_user(CIOCGSESSION)");
		}
	}

	if (*cmd2 == CIOCCRYPT)
	{
		int i = 0;
		i = copy_to_user((const void __user *)arg, cryp1, sizeof(cryp1));
		if (i != 0)
		{
			debug("error with copy_to_user(CIOCCRYPT)");
		}
	}

	// deallocate memory
	kfree(ses);
	kfree(ses1);
	kfree(len_s);
	kfree(op);
	kfree(sess_id);
	kfree(flags);
	kfree(iv);
	kfree(dst);
	kfree(src);
	kfree(key2);
	kfree(key);

	kfree(sess1);
	kfree(cryp1);
	kfree(syscall_type);
	kfree(return_it);
	kfree(cmd2);

	debug("Leaving");

	return ret;
}

static ssize_t crypto_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	debug("Entering");
	debug("Leaving");
	return -EINVAL;
}

static struct file_operations crypto_chrdev_fops =
	{
		.owner = THIS_MODULE,
		.open = crypto_chrdev_open,
		.release = crypto_chrdev_release,
		.read = crypto_chrdev_read,
		.unlocked_ioctl = crypto_chrdev_ioctl,
};

int crypto_chrdev_init(void)
{
	int ret;
	dev_t dev_no;
	unsigned int crypto_minor_cnt = CRYPTO_NR_DEVICES;

	debug("Initializing character device...");
	cdev_init(&crypto_chrdev_cdev, &crypto_chrdev_fops);
	crypto_chrdev_cdev.owner = THIS_MODULE;

	dev_no = MKDEV(CRYPTO_CHRDEV_MAJOR, 0);
	ret = register_chrdev_region(dev_no, crypto_minor_cnt, "crypto_devs");
	if (ret < 0)
	{
		debug("failed to register region, ret = %d", ret);
		goto out;
	}
	ret = cdev_add(&crypto_chrdev_cdev, dev_no, crypto_minor_cnt);
	if (ret < 0)
	{
		debug("failed to add character device");
		goto out_with_chrdev_region;
	}

	debug("Completed successfully");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, crypto_minor_cnt);
out:
	return ret;
}

void crypto_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int crypto_minor_cnt = CRYPTO_NR_DEVICES;

	debug("entering");
	dev_no = MKDEV(CRYPTO_CHRDEV_MAJOR, 0);
	cdev_del(&crypto_chrdev_cdev);
	unregister_chrdev_region(dev_no, crypto_minor_cnt);
	debug("leaving");
}
