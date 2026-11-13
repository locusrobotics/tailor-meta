"""Small S3-backed request ledger, with conditional creation and update."""
import json
from urllib.parse import urlsplit

from botocore.exceptions import ClientError


class State:
    def __init__(self, client, uri, repository):
        location = urlsplit(uri)
        if location.scheme != 's3' or not location.netloc or not location.path.strip('/'):
            raise ValueError('SOURCE_CI_STATE_URI must select a dedicated s3://bucket/prefix')
        self.client, self.bucket = client, location.netloc
        self.prefix = location.path.strip('/') + '/' + repository + '/'

    def get(self, key):
        try:
            value = self.client.get_object(Bucket=self.bucket, Key=self.prefix + key)
        except ClientError as error:
            if error.response['Error']['Code'] in ('NoSuchKey', '404'):
                return None, None
            raise
        return json.loads(value['Body'].read()), value['ETag']

    def put(self, key, value, etag=None):
        condition = {'IfMatch': etag} if etag else {'IfNoneMatch': '*'}
        self.client.put_object(Bucket=self.bucket, Key=self.prefix + key,
                               Body=json.dumps(value, sort_keys=True).encode(), ContentType='application/json',
                               **condition)

    def records(self):
        pages = self.client.get_paginator('list_objects_v2').paginate(
            Bucket=self.bucket, Prefix=self.prefix + 'requests/')
        for page in pages:
            for item in page.get('Contents', []):
                key = item['Key'].removeprefix(self.prefix)
                record, etag = self.get(key)
                if record:
                    yield key, record, etag
