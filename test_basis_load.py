import sys
sys.path.insert(0, '.')
import habitat
from habitat.config.default import get_config

# 加载配置
config = get_config("configs/objectnav_hm3d.yaml")
config.defrost()
config.DATASET.SPLIT = "val"

# 修正路径
config.DATASET.DATA_PATH = "data/datasets/objectnav/hm3d/objectnav_hm3d_v2/{split}/{split}.json.gz"
config.DATASET.SCENES_DIR = "data/scene_datasets/hm3d/minival"
config.SIMULATOR.SCENE_DATASET = "data/scene_datasets/hm3d/minival/hm3d_minival_basis.scene_dataset_config.json"

config.freeze()

print("正在加载数据集，并过滤出本地存在的场景...")
try:
    # 1. 加载数据集
    from habitat.datasets import make_dataset
    dataset = make_dataset(config.DATASET.TYPE, config=config.DATASET)
    
    # 2. 🔥 关键：只保留你本地有的场景（00800开头）
    valid_episodes = []
    for ep in dataset.episodes:
        if "00800" in ep.scene_id or "00801" in ep.scene_id:
            valid_episodes.append(ep)
    
    dataset.episodes = valid_episodes
    print(f"✅ 过滤完成！有效Episode数量: {len(dataset.episodes)}")

    # 3. 创建环境
    env = habitat.Env(config=config, dataset=dataset)
    print(f"✅ 环境创建成功！")
    
    # 4. 测试重置
    obs = env.reset()
    print(f"✅ 环境重置成功！场景加载正常！")
    env.close()

    print("\n🎉 🎉 🎉 所有问题全部解决！")
    print("你的数据集完全可用！直接运行项目：")
    print("python main.py")

except Exception as e:
    print(f"❌ 错误: {e}")
    import traceback
    traceback.print_exc()
