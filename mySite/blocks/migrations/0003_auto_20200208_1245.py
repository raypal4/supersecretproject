# Generated by Django 3.0.3 on 2020-02-08 04:45

from django.db import migrations

import json
from django.contrib.gis.geos import fromstr
from pathlib import Path

DATA_FILENAME = 'data.json'


def load_data(apps, schema_editor):
    blocks = apps.get_model('blocks', 'block')
    jsonfile = Path(__file__).parents[2] / DATA_FILENAME

    with open(str(jsonfile), 'r', encoding='utf8', errors='ignore') as datafile:
        objects = json.load(datafile)
        for obj in objects['elements']:
            try:
                objType = obj['type']
                if objType == 'node':
                    nodeid = obj.get('id')
                    longitude = obj.get('lon', 0)
                    latitude = obj.get('lat', 0)
                    location = fromstr(
                        f'POINT({longitude} {latitude})', srid=4326)
                    blocks(name=nodeid, location=location).save()
            except KeyError:
                pass


class Migration(migrations.Migration):

    dependencies = [
        ('blocks', '0002_auto_20200208_0046'),
    ]

    operations = [
        migrations.RunPython(load_data)
    ]
