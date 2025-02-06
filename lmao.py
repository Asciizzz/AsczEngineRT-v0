from datasets import load_dataset
from transformers import MBartForConditionalGeneration, MBartTokenizer, Trainer, TrainingArguments

# dataset = load_dataset('csv', data_files={'': '', '': ''})

tokenizer = MBartTokenizer.from_pretrained("facebook/mbart-large-cc25")
tokenizer.src_lang = "vi_VN"
tokenizer.tgt_lang = "en_XX"

def preprocess_function(examples):
    inputs = [ex for ex in examples['source_text']]
    targets = [ex for ex in examples['target_text']]
    model_inputs = tokenizer(inputs, max_length=128, truncation=True)
    with tokenizer.as_target_tokenizer():
        labels = tokenizer(targets, max_length=128, truncation=True)
    model_inputs["labels"] = labels["input_ids"]
    return model_inputs

encoded_dataset = dataset.map(preprocess_function, batched=True)

model = MBartForConditionalGeneration.from_pretrained("facebook/mbart-large-cc25")

training_args = TrainingArguments(
    output_dir="output",
    evaluation_strategy="steps",
    per_device_train_batch_size=2,
    per_device_eval_batch_size=2,
    logging_steps=100,
    save_steps=500,
    num_train_epochs=1
)

trainer = Trainer(
    model=model,
    args=training_args,
    train_dataset=encoded_dataset["train"],
    eval_dataset=encoded_dataset["validation"],
)

trainer.train()