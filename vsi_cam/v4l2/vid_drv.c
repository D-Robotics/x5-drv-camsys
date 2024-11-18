// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "utils.h"
#include "video.h"

#include "vid_drv.h"

#define VID_MDEV_NAME       "vs-media"

static struct device_node *get_sensor_node(struct device_node *csi_node)
{
	struct fwnode_handle *csi_fwnode, *endpoint = NULL, *remote_endpoint = NULL;
	struct fwnode_handle *remote_port = NULL, *remote_node = NULL;
	struct device_node *sensor_node = NULL;

	csi_fwnode = of_fwnode_handle(csi_node);
	endpoint = fwnode_graph_get_next_endpoint(csi_fwnode, NULL);
	if (!endpoint)
		return NULL;

	remote_endpoint = fwnode_graph_get_remote_endpoint(endpoint);
	if (!remote_endpoint)
		goto out_put_endpoint;

	remote_port = fwnode_get_parent(remote_endpoint);
	if (!remote_port)
		goto out_put_remote_endpoint;

	remote_node = fwnode_get_parent(remote_port);
	if (!remote_node)
		goto out_put_remote_port;

	sensor_node = to_of_node(remote_node);
	if (!sensor_node)
		goto out_put_remote_node;

out_put_remote_node:
	fwnode_handle_put(remote_node);
out_put_remote_port:
	fwnode_handle_put(remote_port);
out_put_remote_endpoint:
	fwnode_handle_put(remote_endpoint);
out_put_endpoint:
	fwnode_handle_put(endpoint);

	return sensor_node;
}

static int init_subdev_list(struct device *dev,
			    struct v4l2_async_notifier *notifier)
{
	struct device_node *node = NULL, *sensor_node = NULL;
	struct v4l2_async_subdev *asd;
	struct i2c_client *i2c_dev;
	char * const sd_names[] = {
		ISP_DT_NAME,
		VSE_DT_NAME,
		SIF_DT_NAME,
		CSI_DT_NAME,
		GDC_DT_NAME,
	};
	int i = 0;

	while (i < ARRAY_SIZE(sd_names)) {
		while (true) {
			node = of_find_compatible_node(node, NULL, sd_names[i]);
			if (!node)
				break;

			if (!of_device_is_available(node))
				return -1;

			asd = v4l2_async_nf_add_fwnode(notifier,
						       of_fwnode_handle(node),
						       struct v4l2_async_subdev);
			if (IS_ERR(asd))
				return PTR_ERR(asd);

			if (strcmp(sd_names[i], CSI_DT_NAME) == 0) {
				sensor_node = get_sensor_node(node);
				if (!sensor_node)
					continue;

				if (!of_device_is_available(sensor_node))
					return -1;

				i2c_dev = of_find_i2c_device_by_node(sensor_node);
				if (!(i2c_dev && i2c_dev->dev.driver))
					continue;

				asd = v4l2_async_nf_add_fwnode(notifier,
						       of_fwnode_handle(sensor_node),
						       struct v4l2_async_subdev);
				if (IS_ERR(asd))
					return PTR_ERR(asd);

			}
		}
		i++;
	}
	return 0;
}

static int vid_link_notify(struct media_link *link, unsigned int flags,
			   unsigned int notification)
{
	return 0;
}

static const struct media_device_ops vid_mdev_ops = {
	.link_notify = vid_link_notify,
};

static int sd_async_notifier_bound(struct v4l2_async_notifier *notifier,
				   struct v4l2_subdev *sd, struct v4l2_async_subdev *asd)
{
	if (sd->entity.function == MEDIA_ENT_F_CAM_SENSOR)
		return 0;

	struct subdev_node *sn = container_of(sd, struct subdev_node, sd);

	if (sn->async_bound)
		sn->async_bound(sn);
	return 0;
}

static int sd_async_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct vid_device *vid_dev = container_of(notifier, struct vid_device, sd_notifier);
	int rc;

	rc = create_default_links(vid_dev);
	if (rc < 0)
		return rc;

	rc = vid_subdev_set_cap(vid_dev);
	if (rc < 0) {
		destroy_links(vid_dev);
		return rc;
	}

	rc = vid_subdev_init_output_ctx(vid_dev);
	if (rc < 0) {
		destroy_links(vid_dev);
		return rc;
	}

	rc = v4l2_device_register_subdev_nodes(&vid_dev->v4l2_dev);
	if (rc < 0)
		destroy_links(vid_dev);
	return rc;
}

static const struct v4l2_async_notifier_operations sd_async_notifier_ops = {
	.bound = sd_async_notifier_bound,
	.complete = sd_async_notifier_complete,
};

static int vid_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vid_device *vid_dev;
	struct v4l2_device *vdev;
	struct media_device *mdev;
	int rc;

	vid_dev = devm_kzalloc(dev, sizeof(*vid_dev), GFP_KERNEL);
	if (!vid_dev)
		return -ENOMEM;

	INIT_LIST_HEAD(&vid_dev->video_device_list);
	INIT_LIST_HEAD(&vid_dev->m2m_video_device_list);

	mdev = &vid_dev->mdev;
	snprintf(mdev->model, sizeof(mdev->model), "%s", VID_MDEV_NAME);
	snprintf(mdev->bus_info, sizeof(mdev->bus_info), "%s", VID_BUS_INFO);
	mdev->ops = &vid_mdev_ops;
	mdev->dev = dev;
	media_device_init(mdev);

	rc = media_device_register(mdev);
	if (rc < 0) {
		dev_err(dev, "register media device failed (err=%d)\n", rc);
		return rc;
	}

	vdev = &vid_dev->v4l2_dev;
	rc = v4l2_device_register(dev, vdev);
	if (rc < 0)
		return rc;
	vdev->mdev = mdev;

	v4l2_async_nf_init(&vid_dev->sd_notifier);
	vid_dev->sd_notifier.ops = &sd_async_notifier_ops;

	rc = init_subdev_list(dev, &vid_dev->sd_notifier);
	if (rc < 0)
		return rc;

	rc = v4l2_async_nf_register(vdev, &vid_dev->sd_notifier);
	if (rc < 0)
		return rc;

	platform_set_drvdata(pdev, vid_dev);
	dev_dbg(dev, "VS video driver (v4l) probed done\n");
	return 0;
}

static int vid_remove(struct platform_device *pdev)
{
	struct vid_device *vid_dev = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	destroy_links(vid_dev);
	v4l2_async_nf_cleanup(&vid_dev->sd_notifier);
	v4l2_async_nf_unregister(&vid_dev->sd_notifier);
	v4l2_device_unregister(&vid_dev->v4l2_dev);
	media_device_unregister(&vid_dev->mdev);
	media_device_cleanup(&vid_dev->mdev);
	devm_kfree(dev, vid_dev);
	dev_dbg(dev, "VS video driver (v4l) removed\n");
	return 0;
}

static void vid_dev_release(struct device *dev)
{
}

static struct platform_device vid_pdev = {
	.name = VID_DEV_NAME,
	.dev.release = vid_dev_release,
};

static struct platform_driver vid_pdrv = {
	.probe	= vid_probe,
	.remove = vid_remove,
	.driver = {
		.name = VID_DEV_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init vid_init(void)
{
	int rc;

	rc = platform_device_register(&vid_pdev);
	if (rc) {
		dev_err(&vid_pdev.dev,
			"platform device registration failed (err=%d)\n", rc);
		return rc;
	}

	rc = platform_driver_register(&vid_pdrv);
	if (rc) {
		dev_err(&vid_pdev.dev,
			"platform driver registration failed (err=%d)\n", rc);
		platform_driver_unregister(&vid_pdrv);
		return rc;
	}

	return 0;
}

static void __exit vid_exit(void)
{
	platform_driver_unregister(&vid_pdrv);
	platform_device_unregister(&vid_pdev);
}

module_init(vid_init);
module_exit(vid_exit);

MODULE_DESCRIPTION("VeriSilicon Video Driver");
MODULE_AUTHOR("VeriSilicon Camera SW Team");
MODULE_LICENSE("GPL");
MODULE_ALIAS("VeriSilicon-Video");
