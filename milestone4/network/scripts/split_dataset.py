import sys
import os
from random import sample
from tqdm import tqdm
import numpy as np
import json
import argparse
import h5py
from tqdm import tqdm
from glob import glob

def main(args):
    dataset_path = os.path.join('dataset')
    if not os.path.exists(dataset_path):
        raise Exception('dataset path % s does not exist' % dataset_path)
    counter = 0
    dataset_catalog = {}
    dir_temp = os.path.join(dataset_path, 'images')
    all_images =  os.listdir(dir_temp)
    for image_name in all_images:
        image_path = os.path.join(
            'dataset', 'images', image_name)
        label_path = os.path.join(
            'dataset', 'labels', image_name[:-4]+'_label.png')
        dataset_catalog[counter] = {'image_path': image_path, 'label_path':label_path}
        counter +=1

    # split training and evaluation
    train_ratio = args.training_ratio
    if 0<train_ratio<1:
        sample_size = len(dataset_catalog)
        all_indices = list(np.arange(sample_size))
        train_indices = sample(all_indices, int(train_ratio*sample_size))
        eval_indices = list(set(all_indices) - set(train_indices))
        train_indices = [int(x) for x in train_indices]
        eval_indices = [int(x) for x in eval_indices]
        train_catalog = {key:dataset_catalog[key] for key in train_indices}
        eval_catalog = {key:dataset_catalog[key] for key in eval_indices}
        binary_dir = os.path.join('dataset', args.output_dir)
        if not os.path.exists(binary_dir):
            os.makedirs(binary_dir)
        generate_binary_file(train_catalog, 
                             os.path.join(binary_dir, 'train.hdf5'))
        generate_binary_file(eval_catalog, 
                             os.path.join(binary_dir,'eval.hdf5'))
    else:
        raise Exception('training ration must be between (0, 1)')

def generate_binary_file(dataset_catalog, save_path):
    
    hf = h5py.File(save_path, 'a')
    n_samples = len(dataset_catalog.keys())
    print(n_samples)
    dt = h5py.special_dtype(vlen=np.dtype('uint8'))
    image_data = hf.create_dataset('images', (n_samples, ), dtype=dt)
    label_data = hf.create_dataset('labels', (n_samples, ), dtype=dt)
    for idx, data_pair in tqdm(enumerate(dataset_catalog.values())):
        img_f = open(data_pair["image_path"], 'rb')
        img_binary = img_f.read()
        image_data[idx] = np.frombuffer(img_binary, dtype='uint8')
        label_f = open(data_pair["label_path"], 'rb')
        label_binary = label_f.read()
        label_data[idx] = np.frombuffer(label_binary, dtype='uint8')
    hf.close()

if __name__ == '__main__':
    generator_parser = argparse.ArgumentParser(
        description='Split dataset for trainig and evaluation')
    generator_parser.add_argument('--output_dir', type=str, default='')
    generator_parser.add_argument('--training_ratio' ,  type=float, default=0.8,
                                  help='Ratio of trainig over evaluation')
    
    args = generator_parser.parse_args()
    
    main(args)

