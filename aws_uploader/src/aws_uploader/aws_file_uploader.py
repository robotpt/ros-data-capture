#!/usr/bin/env python

import logging
import boto3
import os

from botocore.exceptions import ClientError


class AwsFileUploader:

    def __init__(self, region, default_bucket=None):

        self._default_region = region
        self._s3_client = boto3.client('s3', region_name=self._default_region)

        if default_bucket is None:
            default_bucket = raw_input("What is the name of your bucket?\n>>> ")

        self._create_bucket_if_doesnt_exist(default_bucket, self._default_region)
        self._default_bucket = default_bucket

    # Note: overrides all files with the same name
    def upload_directory_contents_to_aws_directory(
            self,
            directory_path,
            bucket_name=None,
            is_remove_file_on_success=True,
    ):

        if os.path.abspath(directory_path) == os.getcwd():
            raise IOError("Directory selected cannot be directory of this file: '{}'".format(directory_path))
        if self._is_parent_directory(directory_path):
            raise IOError("Directory selected cannot have other directories in it: '{}'".format(directory_path))

        file_names = os.listdir(directory_path)
        for file_name in file_names:
            file_path = os.path.join(directory_path, file_name)
            self.upload_file(
                file_path,
                bucket_name=bucket_name,
                is_remove_file_on_success=is_remove_file_on_success,
            )

    @staticmethod
    def _is_parent_directory(path):
        for item in os.listdir(path):
            if os.path.isdir(os.path.join(path, item)):
                return True
        return False

    def upload_file(self, file_path, bucket_name=None, is_remove_file_on_success=False):

        if bucket_name is None:
            bucket_name = self._default_bucket

        if not os.path.exists(file_path):
            raise ReferenceError("No file '{}' exists".format(file_path))

        try:
            self._s3_client.upload_file(file_path, bucket_name, os.path.basename(file_path))
        except ClientError:
            logging.error("Failed to upload '{}'".format(file_path))
        else:
            logging.info("Successfully uploaded '{}'".format(file_path))
            if is_remove_file_on_success:
                os.remove(file_path)
                logging.info("Deleted file after successful upload: '{}'".format(file_path))

    def get_bucket_names(self):
        return [bucket[u'Name'] for bucket in self._s3_client.list_buckets()[u'Buckets']]

    def set_default_bucket(self, name, region=None):

        if region is None:
            region = self._default_region

        self._create_bucket_if_doesnt_exist(name, region)
        self._default_bucket = name
        logging.info("Default bucket set to '{}'".format(name))

    def _create_bucket_if_doesnt_exist(self, name, region=None):

        if self._is_bucket_exist(name):
            logging.info("Bucket '{}' exists".format(name))
        else:

            if region is None:
                region = self._default_region

            logging.info("Bucket '{}' doesn't exist -- attempting to create".format(name))
            self._create_bucket(
                name=name,
                region=region,
            )
            logging.info("Bucket '{}' created in region '{}'".format(name, region))

    def _is_bucket_exist(self, name):
        return name in self.get_bucket_names()

    def _create_bucket(self, name, region=None):

        if region is None:
            region = self._default_region

        location = {'LocationConstraint': region}
        self._s3_client.create_bucket(
            Bucket=name,
            CreateBucketConfiguration=location,
        )


if __name__ == '__main__':

    import datetime

    temp_dir = 'temp'
    temp_file = 'foo.bar'

    bucket_name_ = "my-bucket-124354731409183749"
    bucket_region_ = "us-west-1"

    aws_uploader = AwsFileUploader(
        region=bucket_region_,
        default_bucket=bucket_name_,
    )

    if not os.path.exists(temp_dir):
        os.mkdir(temp_dir)

    file_path_ = os.path.join(temp_dir, temp_file)
    with open(file_path_, 'w') as f:
        f.write("Some content generated at {}".format(datetime.datetime.now().strftime("%H:%M:%S on %Y-%m-%d")))

    aws_uploader.upload_file(file_path_, is_remove_file_on_success=True)
    os.rmdir(temp_dir)
