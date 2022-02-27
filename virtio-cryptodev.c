/*
 * Virtio Cryptodev Device
 *
 * Implementation of virtio-cryptodev qemu backend device.
 *
 * Dimitris Siakavaras <jimsiak@cslab.ece.ntua.gr>
 * Stefanos Gerangelos <sgerag@cslab.ece.ntua.gr>
 * Konstantinos Papazafeiropoulos <kpapazaf@cslab.ece.ntua.gr>
 *
 */

#include "qemu/osdep.h"
#include "qemu/iov.h"
#include "hw/qdev.h"
#include "hw/virtio/virtio.h"
#include "standard-headers/linux/virtio_ids.h"
#include "hw/virtio/virtio-cryptodev.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <crypto/cryptodev.h>

#define DATA_SIZE 256
#define BLOCK_SIZE 16
#define KEY_SIZE 16 /* AES128 */

static uint64_t get_features(VirtIODevice *vdev, uint64_t features, Error **errp)
{
    DEBUG_IN();
    return features;
}

static void get_config(VirtIODevice *vdev, uint8_t *config_data)
{
    DEBUG_IN();
}

static void set_config(VirtIODevice *vdev, const uint8_t *config_data)
{
    DEBUG_IN();
}

static void set_status(VirtIODevice *vdev, uint8_t status)
{
    DEBUG_IN();
}

static void vser_reset(VirtIODevice *vdev)
{
    DEBUG_IN();
}

static void vq_handle_output(VirtIODevice *vdev, VirtQueue *vq)
{
    VirtQueueElement *elem; //      struct iovec* in_sg; ->used by the driver(guest) to receive data
                            //      struct iovec* out_sg; -> used by the driver(guest) to send data
    unsigned int *syscall_type;

    DEBUG_IN();

    elem = virtqueue_pop(vq, sizeof(VirtQueueElement));
    if (!elem)
    {
        DEBUG("No item to pop from VQ :(");
        return;
    }

    DEBUG("I have got an item from VQ :)");

    syscall_type = elem->out_sg[0].iov_base;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    switch (*syscall_type)
    {
    case VIRTIO_CRYPTODEV_SYSCALL_TYPE_OPEN:
        DEBUG("VIRTIO_CRYPTODEV_SYSCALL_TYPE_OPEN");
        /* ?
        We get the elements from elem, and put them in vdev?
        somewhere here we open crypto dev

        ? */
        int *fd;
        fd = elem->in_sg[0].iov_base;
        *fd = open("/dev/crypto", O_RDWR);
        if (*fd < 0)
        {
            perror("open fucked up");
        }
        break;

    case VIRTIO_CRYPTODEV_SYSCALL_TYPE_CLOSE:
        DEBUG("VIRTIO_CRYPTODEV_SYSCALL_TYPE_CLOSE");
        /* ?
        We get the elements from elem, and put them in vdev?
        somewhere here we close crypto dev
        ? */
        int *cfd, *return_it;
        cfd = elem->out_sg[1].iov_base;
        return_it = elem->in_sg[0].iov_base;

        *return_it = close(*cfd);
        if (*return_it < 0)
        {
            perror("close");
        }

        break;

    case VIRTIO_CRYPTODEV_SYSCALL_TYPE_IOCTL:
        DEBUG("VIRTIO_CRYPTODEV_SYSCALL_TYPE_IOCTL"); // valame mono to id gia na to thimomaste
                                                      /* ?
                                                      We get the elements from elem, and put them in vdev?
                                                      ? */

        //************************output****************************************************************************
        // Aquiring data from buffers(deep copy not here)
        int *pfd;
        unsigned int *name;

        pfd = elem->out_sg[1].iov_base;
        name = elem->out_sg[2].iov_base;

        //************************************input**************************************************************

        int *return_val = 99;                 //->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        return_val = elem->in_sg[0].iov_base; //->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        //**************************************sess kai cryp init**********************************************
        // Initializing structs
        struct session_op sess;
        memset(&sess, 0, sizeof(sess));
        sess.cipher = CRYPTO_AES_CBC;
        sess.keylen = KEY_SIZE;

        struct crypt_op cryp;
        memset(&cryp, 0, sizeof(cryp));

        //***********************************if else***********************************************************
        if (*name == CIOCGSESSION)
        {

            __u8 *key = elem->out_sg[3].iov_base;
            sess.key = *key; ///    pointer here for sure

            __u32 *sess_id = elem->in_sg[1].iov_base;
            sess.ses = *sess_id;

            if ((*return_val = ioctl(*pfd, CIOCGSESSION, &sess)) < 0)
            {
                perror("ioctl(CIOCGSESSION)");
            }
        }
        //***********************************if else***********************************************************

        if (*name == CIOCCRYPT)
        {

            __u16 *cryp_op = elem->out_sg[3].iov_base;
            cryp.op = *cryp_op;

            __u8 *output_msg = elem->out_sg[4].iov_base;
            cryp.src = *output_msg; // -> out   cryp.src= output message

            __u32 *sess_id = elem->out_sg[5].iov_base;
            cryp.ses = *sess_id;

            __u8 *cryp_iv = elem->out_sg[6].iov_base; /* initialization vector for encryption operations */
            cryp.iv = *cryp_iv;

            __u32 *cryp_len = elem->out_sg[7].iov_base;
            cryp.len = *cryp_len;

            __u16 *cryp_flags = elem->out_sg[8].iov_base;
            cryp.flags = *cryp_flags;

            __u8 *input_msg = elem->in_sg[1].iov_base;
            cryp.dst = *input_msg; //->in        cryp.dst=outputmessage

            if ((*return_val = ioctl(*pfd, CIOCCRYPT, &cryp)) < 0)
            {
                perror("ioctl(CIOCCRYPT)");
            }
        }
        //***********************************if else***********************************************************

        if (*name == CIOCFSESSION)
        {

            __u32 *sess_id = elem->out_sg[3].iov_base;
            sess.ses = *sess_id;

            __u8 *key = elem->out_sg[4].iov_base;
            sess.key = *key; ///    pointer here for sure

            if ((*return_val = ioctl(*pfd, CIOCFSESSION, &sess.ses)) < 0)
            {
                perror("ioctl(CIOCFSESSION)");
            }
        }

        break;
        //**********************************end if else***********************************************************

    default:
        DEBUG("Unknown syscall_type");
        break;
    }

    virtqueue_push(vq, elem, 0);
    virtio_notify(vdev, vq);
    g_free(elem);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void virtio_cryptodev_realize(DeviceState *dev, Error **errp)
{
    VirtIODevice *vdev = VIRTIO_DEVICE(dev);

    DEBUG_IN();

    virtio_init(vdev, "virtio-cryptodev", VIRTIO_ID_CRYPTODEV, 0);
    virtio_add_queue(vdev, 128, vq_handle_output);
}

static void virtio_cryptodev_unrealize(DeviceState *dev, Error **errp)
{
    DEBUG_IN();
}

static Property virtio_cryptodev_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void virtio_cryptodev_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    VirtioDeviceClass *k = VIRTIO_DEVICE_CLASS(klass);

    DEBUG_IN();
    dc->props = virtio_cryptodev_properties;
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);

    k->realize = virtio_cryptodev_realize;
    k->unrealize = virtio_cryptodev_unrealize;
    k->get_features = get_features;
    k->get_config = get_config;
    k->set_config = set_config;
    k->set_status = set_status;
    k->reset = vser_reset;
}

static const TypeInfo virtio_cryptodev_info = {
    .name = TYPE_VIRTIO_CRYPTODEV,
    .parent = TYPE_VIRTIO_DEVICE,
    .instance_size = sizeof(VirtCryptodev),
    .class_init = virtio_cryptodev_class_init,
};

static void virtio_cryptodev_register_types(void)
{
    type_register_static(&virtio_cryptodev_info);
}

type_init(virtio_cryptodev_register_types)
