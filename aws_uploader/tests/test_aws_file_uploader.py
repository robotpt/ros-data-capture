#!/usr/bin/env python

import unittest
import mock
import os
import datetime

from aws_uploader.aws_file_uploader import AwsFileUploader, FileBeingWrittenToError


PKG = 'aws_uploader'


class TestAwsFileUploader(unittest.TestCase):

    @mock.patch('aws_uploader.aws_file_uploader.raw_input')
    @mock.patch('aws_uploader.aws_file_uploader.boto3.client')
    def test_constructor_with_all_args(self, mock_aws_client, mock_raw_input):

        foo_region = 'foo-west-1'
        foo_bucket_name = 'foo-bucket'

        mock_aws_client.return_value = mock_aws_client

        aws_uploader = AwsFileUploader(foo_region, foo_bucket_name)

        assert mock_aws_client.call_count == 1
        assert mock_raw_input.call_count == 0
        assert aws_uploader._default_bucket == foo_bucket_name
        assert aws_uploader._default_region == foo_region

    @mock.patch('aws_uploader.aws_file_uploader.raw_input')
    @mock.patch('aws_uploader.aws_file_uploader.boto3.client')
    def test_constructor_with_user_input(self, mock_aws_client, mock_raw_input):

            foo_region = 'foo-west-1'
            foo_bucket_name = 'foo-bucket'

            mock_aws_client.return_value = mock_aws_client
            mock_raw_input.return_value = foo_bucket_name

            aws_uploader = AwsFileUploader(foo_region)

            assert mock_aws_client.call_count == 1
            assert mock_raw_input.call_count == 1
            assert aws_uploader._default_bucket == foo_bucket_name
            assert aws_uploader._default_region == foo_region

    @mock.patch('aws_uploader.aws_file_uploader.boto3.client')
    def test_set_default_bucket(self, _):

        foo_region = 'foo-west-1'
        foo_bucket_name = 'foo-bucket'
        aws_uploader = AwsFileUploader(foo_region, foo_bucket_name)
        assert aws_uploader._default_bucket == foo_bucket_name

        new_foo_bucket = 'new-foo-bucket'
        aws_uploader.set_default_bucket(new_foo_bucket)
        assert aws_uploader._default_bucket == new_foo_bucket

    @mock.patch('aws_uploader.aws_file_uploader.boto3.client')
    def test_upload_and_delete_file(self, _):

        foo_region = 'foo-west-1'
        foo_bucket_name = 'foo-bucket'

        aws_uploader = AwsFileUploader(foo_region, foo_bucket_name)

        temp_dir = 'temp'
        temp_file = 'foo.bar'
        file_path_ = os.path.join(temp_dir, temp_file)

        assert not os.path.exists(temp_dir)
        os.mkdir(temp_dir)

        with open(file_path_, 'w') as f:
            f.write("Some content generated at {}".format(datetime.datetime.now().strftime("%H:%M:%S on %Y-%m-%d")))

        assert os.path.exists(file_path_)
        aws_uploader.upload_file(file_path_, is_remove_file_on_upload=False)
        assert os.path.exists(file_path_)
        aws_uploader.upload_file(file_path_, is_remove_file_on_upload=True)
        assert not os.path.exists(file_path_)
        os.rmdir(temp_dir)

    @mock.patch('aws_uploader.aws_file_uploader.boto3.client')
    def test_raise_error_on_upload_file_that_doesnt_exist(self, _):

        foo_region = 'foo-west-1'
        foo_bucket_name = 'foo-bucket'

        aws_uploader = AwsFileUploader(foo_region, foo_bucket_name)

        not_created_dir = 'temp'
        not_created_file = 'foo.bar'
        not_created_file_path = os.path.join(not_created_dir, not_created_file)
        assert not os.path.exists(not_created_file_path)

        self.assertRaises(
            ReferenceError,
            aws_uploader.upload_file,
            not_created_file_path,
        )

    @mock.patch('aws_uploader.aws_file_uploader.boto3.client')
    def test_send_all_files_in_a_directory(self, mock_aws_client):

        foo_region = 'foo-west-1'
        foo_bucket_name = 'foo-bucket'

        mock_aws_client.return_value = mock_aws_client
        aws_uploader = AwsFileUploader(foo_region, foo_bucket_name)
        assert mock_aws_client.upload_file.call_count == 0

        temp_dir = 'temp'
        assert not os.path.exists(temp_dir)
        os.mkdir(temp_dir)

        assert len(os.listdir(temp_dir)) == 0

        files_created = []
        temp_file_base_name = 'foo'
        temp_ext = '.bar'
        num_files = 10
        for i in range(num_files):
            file_path_ = os.path.join(temp_dir, temp_file_base_name + str(i) + temp_ext)
            with open(file_path_, 'w') as f:
                f.write("Some content generated at {}".format(datetime.datetime.now().strftime("%H:%M:%S on %Y-%m-%d")))
            files_created.append(file_path_)
        assert len(os.listdir(temp_dir)) == num_files

        aws_uploader.upload_directory_contents_to_aws_directory(
            directory_path=temp_dir,
            is_remove_file_on_upload=False
        )
        assert len(os.listdir(temp_dir)) == num_files
        assert mock_aws_client.upload_file.call_count == num_files

        aws_uploader.upload_directory_contents_to_aws_directory(
            directory_path=temp_dir,
            is_remove_file_on_upload=True
        )
        assert len(os.listdir(temp_dir)) == 0
        assert mock_aws_client.upload_file.call_count == 2*num_files

        # cleanup created directory
        os.rmdir(temp_dir)

    @mock.patch('aws_uploader.aws_file_uploader.boto3.client')
    def test_that_given_directory_cant_be_local_directory(self, _):

        foo_region = 'foo-west-1'
        foo_bucket_name = 'foo-bucket'
        aws_uploader = AwsFileUploader(foo_region, foo_bucket_name)

        temp_dir = '.'
        self.assertRaises(
            IOError,
            aws_uploader.upload_directory_contents_to_aws_directory,
            directory_path=temp_dir,
        )

    @mock.patch('aws_uploader.aws_file_uploader.boto3.client')
    def test_that_given_directory_cant_have_directories_in_it(self, _):

        foo_region = 'foo-west-1'
        foo_bucket_name = 'foo-bucket'
        aws_uploader = AwsFileUploader(foo_region, foo_bucket_name)

        # Create nested directories
        temp_dir = 'temp'
        assert not os.path.exists(temp_dir)
        os.mkdir(temp_dir)
        for i in range(10):
            new_dir_path = os.path.join(temp_dir, "new-dir" + str(i))
            os.mkdir(new_dir_path)

        self.assertRaises(
            IOError,
            aws_uploader.upload_directory_contents_to_aws_directory,
            directory_path=temp_dir,
        )

        # Clean up created directories
        for f in os.listdir(temp_dir):
            os.rmdir(os.path.join(temp_dir, f))
        os.rmdir(temp_dir)

    def test_is_parent_dir(self):

        temp_dir = 'temp'
        assert not os.path.exists(temp_dir)
        os.mkdir(temp_dir)

        assert len(os.listdir(temp_dir)) == 0
        assert not AwsFileUploader._is_parent_directory(temp_dir)

        for i in range(10):
            new_dir_path = os.path.join(temp_dir, "new-dir" + str(i))
            os.mkdir(new_dir_path)
            assert AwsFileUploader._is_parent_directory(temp_dir)

        # Clean up created directories
        for f in os.listdir(temp_dir):
            os.rmdir(os.path.join(temp_dir, f))
        os.rmdir(temp_dir)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_aws_file_uploader', TestAwsFileUploader)
