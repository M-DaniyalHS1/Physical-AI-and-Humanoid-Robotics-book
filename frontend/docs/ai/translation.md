---
sidebar_position: 3
title: "AI Translation"
---

# AI Translation for Physical AI & Humanoid Robotics

This section covers AI-powered translation capabilities for Physical AI and Humanoid Robotics applications, with a focus on supporting the local language (Urdu) for Panaversity users. Translation enables broader access to educational content and enhances human-robot interaction in multilingual environments.

## Translation Architecture for Robotics

### Overview of AI Translation in Robotics

AI translation in robotics involves:
- Real-time translation of user commands and robot responses
- Translation of educational content for different languages
- Multimodal translation (text, speech, and gesture)
- Context-aware translation considering robotics domain terminology

```python
from typing import Dict, List, Optional, Tuple
import asyncio
import logging

class TranslationManager:
    def __init__(self):
        self.translation_cache = {}
        self.supported_languages = {
            'en': 'English',
            'ur': 'Urdu',
            'hi': 'Hindi',
            'es': 'Spanish'
        }
        self.translation_engine = self._initialize_translation_engine()

    def _initialize_translation_engine(self):
        """Initialize the translation engine"""
        # This would typically initialize an actual translation model
        # For example, using Hugging Face transformers, OpenAI, etc.
        return TranslationEngine()

    def translate_text(self,
                      text: str,
                      source_lang: str = 'en',
                      target_lang: str = 'ur',
                      domain: str = 'robotics') -> str:
        """Translate text with domain-specific terminology"""
        cache_key = f"{text}_{source_lang}_{target_lang}_{domain}"

        # Check cache first
        if cache_key in self.translation_cache:
            return self.translation_cache[cache_key]

        # Perform translation
        translated_text = self.translation_engine.translate(
            text=text,
            source_lang=source_lang,
            target_lang=target_lang,
            domain=domain
        )

        # Store in cache
        self.translation_cache[cache_key] = translated_text

        return translated_text
```

## Urdu Translation Implementation

### Urdu Language Specifics

Urdu translation presents unique challenges and opportunities:

1. **Naskh script**: Right-to-left writing system
2. **Cultural context**: Different cultural concepts and expressions
3. **Technical terminology**: Need for appropriate robotics/technology terms
4. **Formality levels**: Different registers for various contexts

```python
import unicodedata
from transformers import MarianMTModel, MarianTokenizer
import torch

class UrduTranslationEngine:
    def __init__(self):
        self.models = {}
        self.tokenizers = {}
        self.domain_specialized_terms = self._load_domain_terms()

        # Initialize Urdu translation models
        self._load_urdu_models()

    def _load_domain_terms(self) -> Dict[str, Dict[str, str]]:
        """Load domain-specific terminology for robotics"""
        return {
            'robotics': {
                'robot': 'روبوٹ',
                'navigation': 'ہدایت',
                'manipulation': 'ہاتھ',
                'sensor': 'سینسر',
                'actuator': 'ایکچو ایٹر',
                'algorithm': 'الگورتھم',
                'perception': 'احساس',
                'planning': ' منصوبہ بندی',
                'control': 'کنٹرول',
                'locomotion': 'چلنے کی صلاحیت',
                'articulation': 'م joint',
                'gripper': 'گرپر',
                'end effector': 'آخری کارکن',
                'kinematics': 'کنيماتکس',
                'dynamics': 'ڈائنامکس',
                'trajectory': 'رخ',
                'waypoint': 'راہ کا پتھر',
                'pose': 'پوز',
                'orientation': 'سمت',
                'position': 'مقام',
                'velocity': 'رفعت',
                'acceleration': 'تیزی',
                'force': 'قوت',
                'torque': 'ٹورک',
                'feedback': 'ردعمل',
                'autonomous': 'خودکار',
                'teleoperation': 'دور کاری',
                'haptic': 'ہیپٹک',
                'tactile': 'چھونے والا',
                'vision': 'وژن',
                'lidar': 'لیزر سینسر',
                'imu': 'انرجی میٹر',
                'gps': 'جی پی ایس',
                'mapping': 'نقشہ کاری',
                'slam': 'SLAM',
                'localization': 'مقام کاری',
                'path planning': 'راہ کی منصوبہ بندی'
            }
        }

    def _load_urdu_models(self):
        """Load translation models for Urdu"""
        # Note: In practice, you would load actual models
        # These are placeholder model names
        try:
            # Load English to Urdu model
            self.models['en_ur'] = MarianMTModel.from_pretrained('Helsinki-NLP/opus-mt-en-ur')
            self.tokenizers['en_ur'] = MarianTokenizer.from_pretrained('Helsinki-NLP/opus-mt-en-ur')

            # Load Urdu to English model
            self.models['ur_en'] = MarianMTModel.from_pretrained('Helsinki-NLP/opus-mt-ur-en')
            self.tokenizers['ur_en'] = MarianTokenizer.from_pretrained('Helsinki-NLP/opus-mt-ur-en')

        except Exception as e:
            logging.warning(f"Could not load translation models: {e}")
            # Fallback to a simpler implementation
            self.models['en_ur'] = None
            self.tokenizers['en_ur'] = None

    def translate_urdu(self, text: str, source_lang: str = 'en') -> str:
        """Translate text to/from Urdu with domain-specific terms"""
        target_lang = 'ur' if source_lang == 'en' else 'en'
        model_key = f"{source_lang}_{target_lang}"

        if model_key in self.models and self.models[model_key]:
            return self._translate_with_model(text, source_lang, target_lang)
        else:
            # Fallback translation
            return self._fallback_translation(text, source_lang)

    def _translate_with_model(self, text: str, source_lang: str, target_lang: str) -> str:
        """Translate using pre-trained models"""
        model_key = f"{source_lang}_{target_lang}"
        tokenizer = self.tokenizers[model_key]
        model = self.models[model_key]

        # Tokenize input
        inputs = tokenizer(text, return_tensors="pt", padding=True, truncation=True)

        # Generate translation
        with torch.no_grad():
            outputs = model.generate(**inputs)

        # Decode output
        translated = tokenizer.decode(outputs[0], skip_special_tokens=True)

        return translated

    def _fallback_translation(self, text: str, source_lang: str) -> str:
        """Fallback translation method"""
        # For demo purposes, return the same text with a note
        return f"[TRANSLATION NOT AVAILABLE] {text}"
```

## Translation Cache and Performance

### Optimized Translation Caching

```python
from collections import OrderedDict
import hashlib
import time

class TranslationCache:
    def __init__(self, max_size: int = 1000, ttl: int = 3600):
        self.cache = OrderedDict()
        self.max_size = max_size
        self.ttl = ttl  # Time-to-live in seconds

    def _generate_key(self, text: str, source_lang: str, target_lang: str, domain: str) -> str:
        """Generate a cache key for the translation request"""
        key_string = f"{text}_{source_lang}_{target_lang}_{domain}"
        return hashlib.md5(key_string.encode()).hexdigest()

    def get(self, text: str, source_lang: str, target_lang: str, domain: str) -> Optional[str]:
        """Get cached translation"""
        key = self._generate_key(text, source_lang, target_lang, domain)

        if key in self.cache:
            cached_result, timestamp = self.cache[key]

            # Check if cache is still valid
            if time.time() - timestamp < self.ttl:
                # Move to end (LRU behavior)
                del self.cache[key]
                self.cache[key] = (cached_result, timestamp)
                return cached_result
            else:
                # Remove expired entry
                del self.cache[key]

        return None

    def put(self, text: str, source_lang: str, target_lang: str, domain: str, result: str):
        """Put translation result in cache"""
        key = self._generate_key(text, source_lang, target_lang, domain)

        # Remove oldest entries if cache is full
        while len(self.cache) >= self.max_size:
            self.cache.popitem(last=False)

        self.cache[key] = (result, time.time())

    def invalidate(self, pattern: str = None):
        """Invalidate cache entries based on pattern"""
        if pattern is None:
            self.cache.clear()
        else:
            # This would implement pattern-based invalidation
            # For simplicity, clearing all in this example
            self.cache.clear()
```

## Content Translation for Educational Materials

### Translating Textbook Content

```python
from dataclasses import dataclass
from enum import Enum

class TranslationStatus(Enum):
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"

@dataclass
class TranslationJob:
    id: str
    content_id: str
    source_lang: str
    target_lang: str
    content: str
    status: TranslationStatus = TranslationStatus.PENDING
    timestamp: float = None
    translated_content: str = None
    error_message: str = None

class ContentTranslationService:
    def __init__(self, translation_engine: UrduTranslationEngine):
        self.translation_engine = translation_engine
        self.translation_cache = TranslationCache()
        self.translation_jobs = {}
        self.domain_specialized_terms = self._load_domain_terms()

    def _load_domain_terms(self) -> Dict[str, Dict[str, str]]:
        """Load domain-specific terminology"""
        return {
            "robotics": {
                "physical ai": "فزیکل اے آئی",
                "humanoid robotics": "ہیومنوائڈ روبوٹس",
                "robot operating system": "روبوٹ آپریٹنگ سسٹم",
                "simulation": "سمولیشن",
                "gazebo": "گزیبو",
                "unity": "یونیٹی",
                "nvidia isaac": "این ویڈیا ایسیک",
                "vision-language-action": "وژن-زبان-کارروائی",
                "vla": "وی ایل اے",
                "reinforcement learning": "ری اینفارسمنٹ لرننگ",
                "deep learning": "گہرائ لرننگ",
                "computer vision": "کمپیوٹر وژن",
                "natural language processing": "نیچرل لینگویج پروسیسنگ",
                "motion planning": "موشن پلاننگ",
                "path planning": "پاتھ پلاننگ",
                "collision detection": "کولیژن ڈیٹیکشن",
                "inverse kinematics": "ان ورس کنیمیٹکس",
                "forward kinematics": "فارورڈ کنیمیٹکس",
                "control systems": "کنٹرول سسٹم",
                "sensors and actuators": "سینسرز اور ایکچو ایٹرز",
                "feedback control": "فریقیک بیک کنٹرول"
            }
        }

    def translate_content(self,
                         content_id: str,
                         text: str,
                         target_lang: str = 'ur',
                         domain: str = 'robotics') -> str:
        """Translate educational content with caching"""
        # Check cache first
        cached_result = self.translation_cache.get(
            text=text,
            source_lang='en',
            target_lang=target_lang,
            domain=domain
        )

        if cached_result:
            return cached_result

        # Apply domain-specific terminology
        processed_text = self._apply_domain_terms(text, domain)

        # Perform translation
        translated_text = self.translation_engine.translate_urdu(
            processed_text,
            source_lang='en'
        )

        # Store in cache
        self.translation_cache.put(
            text=text,
            source_lang='en',
            target_lang=target_lang,
            domain=domain,
            result=translated_text
        )

        return translated_text

    def _apply_domain_terms(self, text: str, domain: str) -> str:
        """Apply domain-specific terminology to text"""
        if domain not in self.domain_specialized_terms:
            return text

        domain_terms = self.domain_specialized_terms[domain]
        processed_text = text

        # Replace technical terms (case-insensitive)
        for english_term, urdu_term in domain_terms.items():
            processed_text = processed_text.replace(
                english_term.lower(),
                f"[{urdu_term} - {english_term}]"
            )
            processed_text = processed_text.replace(
                english_term.capitalize(),
                f"[{urdu_term} - {english_term}]"
            )

        return processed_text

    def translate_document(self,
                          document: Dict[str, str],
                          target_lang: str = 'ur',
                          domain: str = 'robotics') -> Dict[str, str]:
        """Translate an entire document with structure preserved"""
        translated_doc = {}

        for section, content in document.items():
            if isinstance(content, str):
                translated_doc[section] = self.translate_content(
                    content_id=section,
                    text=content,
                    target_lang=target_lang,
                    domain=domain
                )
            elif isinstance(content, dict):
                # Recursively translate nested content
                translated_doc[section] = self.translate_document(
                    content,
                    target_lang,
                    domain
                )
            else:
                translated_doc[section] = content

        return translated_doc
```

## Real-Time Translation for Chatbots

### Integration with AI Chatbot for Urdu Support

```python
class MultilingualChatbot:
    def __init__(self, translation_service: ContentTranslationService,
                 base_chatbot):
        self.translation_service = translation_service
        self.base_chatbot = base_chatbot
        self.active_language = 'en'  # Default language

    def process_message(self, message: str, user_language: str = 'en') -> str:
        """Process message with language support"""
        # Set active language
        self.active_language = user_language

        # If user is speaking in Urdu, translate to English for processing
        if user_language == 'ur':
            english_message = self.translation_service.translation_engine.translate_urdu(
                message,
                source_lang='ur'
            )
            response = self.base_chatbot.process(english_message)
        else:
            response = self.base_chatbot.process(message)

        # Translate response back to user's language if needed
        if self.active_language != 'en':
            # Translate the response back to the user's language
            user_response = self.translation_service.translation_engine.translate_urdu(
                response,
                source_lang='en'
            )
            return user_response
        else:
            return response

    def toggle_language(self, target_lang: str) -> str:
        """Toggle between languages"""
        if target_lang == self.active_language:
            return f"Already using {target_lang}"

        self.active_language = target_lang
        welcome_msg = {
            'ur': "خوش آمدید! آپ اب اردو میں چیٹ کر سکتے ہیں۔",
            'en': "Welcome! You can now chat in English.",
            'hi': "स्वागत है! अब आप हिंदी में चैट कर सकते हैं।"
        }

        if target_lang in welcome_msg:
            return self._translate_if_needed(welcome_msg[target_lang], target_lang)

        return f"Language switched to {target_lang}"

    def _translate_if_needed(self, text: str, target_lang: str) -> str:
        """Translate text if the target language is different from English"""
        if target_lang == 'en':
            return text
        else:
            return self.translation_service.translation_engine.translate_urdu(
                text,
                source_lang='en' if target_lang != 'en' else target_lang
            )
```

## Translation Quality and Evaluation

### Quality Assessment Framework

```python
import numpy as np
from sklearn.metrics import accuracy_score
import sacrebleu

class TranslationQualityEvaluator:
    def __init__(self):
        self.translation_quality_scores = {}
        self.bleu_scores = {}

    def evaluate_translation_quality(self,
                                   original_text: str,
                                   translated_text: str,
                                   reference_translation: str = None) -> Dict[str, float]:
        """Evaluate quality of translation"""
        metrics = {}

        # If reference translation is provided, calculate BLEU score
        if reference_translation:
            try:
                bleu_score = sacrebleu.sentence_bleu(
                    hypothesis=translated_text,
                    references=[reference_translation]
                )
                metrics['bleu_score'] = bleu_score.score
            except:
                metrics['bleu_score'] = 0.0

        # Calculate character-level similarity (simple metric)
        similarity = self._calculate_similarity(original_text, translated_text)
        metrics['character_similarity'] = similarity

        # Language detection accuracy
        try:
            import langdetect
            detected_lang = langdetect.detect(translated_text)
            metrics['correct_language'] = 1.0 if detected_lang.startswith('ur') else 0.0
        except:
            metrics['correct_language'] = 0.5  # Neutral score if detection fails

        return metrics

    def _calculate_similarity(self, text1: str, text2: str) -> float:
        """Calculate similarity between two texts"""
        # Simple character-based similarity
        set1 = set(text1.lower())
        set2 = set(text2.lower())

        intersection = len(set1.intersection(set2))
        union = len(set1.union(set2))

        return intersection / union if union > 0 else 0.0

    def evaluate_batch_translations(self,
                                  original_texts: List[str],
                                  translated_texts: List[str],
                                  reference_texts: List[str] = None) -> Dict[str, float]:
        """Evaluate a batch of translations"""
        all_metrics = {
            'bleu_scores': [],
            'character_similarities': [],
            'language_accuracy': []
        }

        for i, (orig, trans) in enumerate(zip(original_texts, translated_texts)):
            ref = reference_texts[i] if reference_texts else None
            metrics = self.evaluate_translation_quality(orig, trans, ref)

            all_metrics['bleu_scores'].append(metrics.get('bleu_score', 0.0))
            all_metrics['character_similarities'].append(metrics.get('character_similarity', 0.0))
            all_metrics['language_accuracy'].append(metrics.get('correct_language', 0.5))

        # Calculate averages
        avg_metrics = {
            'avg_bleu_score': np.mean(all_metrics['bleu_scores']),
            'avg_character_similarity': np.mean(all_metrics['character_similarities']),
            'avg_language_accuracy': np.mean(all_metrics['language_accuracy']),
        }

        return avg_metrics
```

## ROS 2 Integration for Translation Services

### Translation Service Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ai_msgs.srv import TranslateText
from ai_msgs.msg import TranslationResult

class TranslationNode(Node):
    def __init__(self):
        super().__init__('translation_node')

        # Service for text translation
        self.translation_service = self.create_service(
            TranslateText, 'translate_text', self.translate_text_callback
        )

        # Publisher for translation results
        self.result_publisher = self.create_publisher(
            TranslationResult, 'translation_result', 10
        )

        # Initialize translation components
        self.urdu_engine = UrduTranslationEngine()
        self.translation_service_impl = ContentTranslationService(self.urdu_engine)
        self.quality_evaluator = TranslationQualityEvaluator()

    def translate_text_callback(self, request, response):
        """Callback for translation service"""
        try:
            # Perform translation
            translated_text = self.translation_service_impl.translate_content(
                content_id="service_request",
                text=request.text,
                target_lang=request.target_language,
                domain=request.domain
            )

            # Evaluate quality (if reference provided)
            if request.reference_translation:
                quality_metrics = self.quality_evaluator.evaluate_translation_quality(
                    original_text=request.text,
                    translated_text=translated_text,
                    reference_translation=request.reference_translation
                )

                response.quality_score = quality_metrics.get('bleu_score', 0.0)

            # Set response
            response.translated_text = translated_text
            response.success = True

            # Publish result for other nodes to use
            result_msg = TranslationResult()
            result_msg.original_text = request.text
            result_msg.translated_text = translated_text
            result_msg.source_language = request.source_language
            result_msg.target_language = request.target_language
            result_msg.timestamp = self.get_clock().now().to_msg()

            self.result_publisher.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f"Translation error: {e}")
            response.success = False
            response.error_message = str(e)

        return response
```

## Performance Optimization

### Async Translation Processing

```python
import asyncio
from concurrent.futures import ThreadPoolExecutor

class AsyncTranslationProcessor:
    def __init__(self, translation_engine, max_workers=4):
        self.translation_engine = translation_engine
        self.executor = ThreadPoolExecutor(max_workers=max_workers)

    async def translate_batch_async(self,
                                   texts: List[str],
                                   source_lang: str = 'en',
                                   target_lang: str = 'ur') -> List[str]:
        """Asynchronously translate a batch of texts"""
        loop = asyncio.get_event_loop()

        tasks = [
            loop.run_in_executor(
                self.executor,
                self.translation_engine.translate_urdu,
                text,
                source_lang
            )
            for text in texts
        ]

        results = await asyncio.gather(*tasks, return_exceptions=True)

        # Handle any exceptions in results
        processed_results = []
        for result in results:
            if isinstance(result, Exception):
                processed_results.append(f"[ERROR: {str(result)}]")
            else:
                processed_results.append(result)

        return processed_results

    async def translate_stream(self,
                              text_stream: List[str],
                              source_lang: str = 'en',
                              target_lang: str = 'ur') -> List[str]:
        """Translate a stream of texts with batching for performance"""
        batch_size = 10
        all_results = []

        for i in range(0, len(text_stream), batch_size):
            batch = text_stream[i:i + batch_size]
            batch_results = await self.translate_batch_async(
                batch, source_lang, target_lang
            )
            all_results.extend(batch_results)

        return all_results
```

This implementation provides a comprehensive framework for AI translation in Physical AI and Humanoid Robotics applications, with special focus on Urdu support for Panaversity users. The system includes caching, quality evaluation, real-time processing, and ROS 2 integration capabilities.