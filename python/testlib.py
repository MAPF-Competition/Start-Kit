import torch


def test_torch():
    if torch.cuda.is_available():
        device = torch.device("cuda")
        # torch.cuda.set_device(device)
        print("GPU is available.")
    else:
        device = torch.device("cpu")
        print("GPU is not available. Using CPU instead.")
    # Create a tensor and move it to the appropriate device
    # x = torch.tensor([1, 2, 3])

    print("to device gpu")
    # device = torch.device("cpu")
    # x = x.to(device)
    x=torch.tensor([1,2,3],device=device)
    print("completed",x)
    # Define a model and move it to the appropriate device
    # model = MyModel()
    # model = model.to(device)