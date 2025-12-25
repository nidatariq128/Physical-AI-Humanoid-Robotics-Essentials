import React, { createContext, useContext, useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

// Default English translations
const defaultTranslations = {
  en: {
    // MRI ChatGPT UI
    'mri.assistant.welcome': 'Hello! I\'m your MRI Book Assistant. Ask me anything about MRI technology, principles, applications, or related topics.',
    'mri.assistant.newChat': '+ New Chat',
    'mri.assistant.title': 'MRI Book Assistant',
    'mri.assistant.about': 'About this Assistant',
    'mri.assistant.description': 'This AI assistant has access to the complete MRI book content and can answer questions about:',
    'mri.assistant.mriPhysics': 'MRI Physics and Principles',
    'mri.assistant.imagingTechniques': 'Imaging Techniques',
    'mri.assistant.clinicalApplications': 'Clinical Applications',
    'mri.assistant.imageInterpretation': 'Image Interpretation',
    'mri.assistant.safetyProtocols': 'Safety Protocols',
    'mri.assistant.placeholder': 'Ask a question about MRI...',
    'mri.assistant.thinking': 'Thinking',
    'mri.assistant.error': 'Sorry, I encountered an error: ',
    'mri.assistant.noRelevantInfo': "I couldn't find any relevant information to answer your question. Please try rephrasing your question or ask about a different topic related to MRI.",

    // General chatbot
    'chatbot.welcome': 'Hello! I\'m your AI assistant for the AI Robotics Book. Ask me anything about the content!',
    'chatbot.placeholder.default': 'Ask a question about the documentation...',
    'chatbot.placeholder.context': 'Ask about the selected text...',
    'chatbot.context': 'Context: "',
    'chatbot.sources': 'Sources:',
    'chatbot.close': 'Close chat',
    'chatbot.open': 'Open chatbot',
    'chatbot.title': 'AI Assistant',
  },
  es: {
    // Spanish translations
    'mri.assistant.welcome': '¡Hola! Soy tu asistente del libro de MRI. Pregúntame lo que sea sobre tecnología de MRI, principios, aplicaciones o temas relacionados.',
    'mri.assistant.newChat': '+ Nuevo Chat',
    'mri.assistant.title': 'Asistente del Libro de MRI',
    'mri.assistant.about': 'Acerca de este Asistente',
    'mri.assistant.description': 'Este asistente de IA tiene acceso al contenido completo del libro de MRI y puede responder preguntas sobre:',
    'mri.assistant.mriPhysics': 'Física y Principios de MRI',
    'mri.assistant.imagingTechniques': 'Técnicas de Imagen',
    'mri.assistant.clinicalApplications': 'Aplicaciones Clínicas',
    'mri.assistant.imageInterpretation': 'Interpretación de Imágenes',
    'mri.assistant.safetyProtocols': 'Protocolos de Seguridad',
    'mri.assistant.placeholder': 'Haz una pregunta sobre MRI...',
    'mri.assistant.thinking': 'Pensando',
    'mri.assistant.error': 'Lo siento, encontré un error: ',
    'mri.assistant.noRelevantInfo': "No pude encontrar información relevante para responder tu pregunta. Por favor intenta reformular tu pregunta o preguntar sobre un tema diferente relacionado con MRI.",

    // General chatbot
    'chatbot.welcome': '¡Hola! Soy tu asistente de IA para el libro de Robótica de IA. Pregúntame lo que sea sobre el contenido.',
    'chatbot.placeholder.default': 'Haz una pregunta sobre la documentación...',
    'chatbot.placeholder.context': 'Pregunta sobre el texto seleccionado...',
    'chatbot.context': 'Contexto: "',
    'chatbot.sources': 'Fuentes:',
    'chatbot.close': 'Cerrar chat',
    'chatbot.open': 'Abrir chatbot',
    'chatbot.title': 'Asistente de IA',
  },
  ar: {
    // Arabic translations
    'mri.assistant.welcome': 'مرحباً! أنا مساعدك لكتاب التصوير بالرنين المغناطيسي. اسألي عن أي شيء بخصوص تكنولوجيا التصوير بالرنين المغناطيسي أو المبادئ أو التطبيقات أو المواضيع ذات الصلة.',
    'mri.assistant.newChat': '+ محادثة جديدة',
    'mri.assistant.title': 'مساعد كتاب التصوير بالرنين المغناطيسي',
    'mri.assistant.about': 'حول هذا المساعد',
    'mri.assistant.description': 'هذا المساعد الاصطناعي لديه إمكانية الوصول إلى محتوى كتاب التصوير بالرنين المغناطيسي الكامل ويمكنه الإجابة عن الأسئلة حول:',
    'mri.assistant.mriPhysics': 'الفيزياء والمبادئ',
    'mri.assistant.imagingTechniques': 'تقنيات التصوير',
    'mri.assistant.clinicalApplications': 'التطبيقات السريرية',
    'mri.assistant.imageInterpretation': 'تفسير الصور',
    'mri.assistant.safetyProtocols': 'بروتوكولات السلامة',
    'mri.assistant.placeholder': 'اطرح سؤالاً حول التصوير بالرنين المغناطيسي...',
    'mri.assistant.thinking': 'يفكر',
    'mri.assistant.error': 'عذراً، واجهت خطأ: ',
    'mri.assistant.noRelevantInfo': "لم أتمكن من العثور على معلومات ذات صلة للإجابة على سؤالك. يرجى محاولة إعادة صياغة سؤالك أو طرح سؤال حول موضوع مختلف متعلق بالرنين المغناطيسي.",

    // General chatbot
    'chatbot.welcome': 'مرحباً! أنا مساعدك الاصطناعي لكتاب روبوتات الذكاء الاصطناعي. اسألي عن أي شيء بخصوص المحتوى!',
    'chatbot.placeholder.default': 'اطرح سؤالاً حول التوثيق...',
    'chatbot.placeholder.context': 'اسألي حول النص المحدد...',
    'chatbot.context': 'السياق: "',
    'chatbot.sources': 'المصادر:',
    'chatbot.close': 'إغلاق المحادثة',
    'chatbot.open': 'فتح مساعد المحادثة',
    'chatbot.title': 'مساعد الذكاء الاصطناعي',
  },
  ur: {
    // Urdu translations
    'mri.assistant.welcome': 'ہیلو! میں آپ کا ایم آر آئی کتاب کا اسسٹنٹ ہوں۔ ایم آر آئی ٹیکنالوجی، اصولوں، ایپلی کیشنز یا متعلقہ موضوعات کے بارے میں مجھ سے کچھ بھی پوچھیں۔',
    'mri.assistant.newChat': '+ نیا چیٹ',
    'mri.assistant.title': 'ایم آر آئی کتاب اسسٹنٹ',
    'mri.assistant.about': 'اس اسسٹنٹ کے بارے میں',
    'mri.assistant.description': 'اس آرٹیفیشل انٹیلی جنس اسسٹنٹ کے پاس مکمل ایم آر آئی کتاب کے مواد تک رسائی ہے اور یہ درج ذیل کے بارے میں سوالات کا جواب دے سکتا ہے:',
    'mri.assistant.mriPhysics': 'ایم آر آئی فزکس اور اصول',
    'mri.assistant.imagingTechniques': 'امیجنگ ٹیکنیکس',
    'mri.assistant.clinicalApplications': 'کلینیکل ایپلی کیشنز',
    'mri.assistant.imageInterpretation': 'امیج کی تشریح',
    'mri.assistant.safetyProtocols': 'سیفٹی پروٹوکولز',
    'mri.assistant.placeholder': 'ایم آر آئی کے بارے میں سوال پوچھیں...',
    'mri.assistant.thinking': 'سوچ رہا ہے',
    'mri.assistant.error': 'معذرت، مجھے ایک خرابی کا سامنا کرنا پڑا: ',
    'mri.assistant.noRelevantInfo': "آپ کے سوال کا جواب دینے کے لیے میں کوئی متعلقہ معلومات نہیں تلاش کر سکا۔ براہ کرم اپنا سوال دوبارہ بیان کریں یا ایم آر آئی سے متعلق کوئی اور موضوع پر سوال کریں۔",

    // General chatbot
    'chatbot.welcome': 'ہیلو! میں آپ کا ای آئی روبوٹکس کتاب کا آرٹیفیشل انٹیلی جنس اسسٹنٹ ہوں۔ مواد کے بارے میں کچھ بھی پوچھیں!',
    'chatbot.placeholder.default': 'دستاویزات کے بارے میں سوال کریں...',
    'chatbot.placeholder.context': 'منتخب کردہ متن کے بارے میں پوچھیں...',
    'chatbot.context': 'سیاق: "',
    'chatbot.sources': 'ماخذ:',
    'chatbot.close': 'چیٹ بند کریں',
    'chatbot.open': 'چیٹ بوٹ کھولیں',
    'chatbot.title': 'آرٹیفیشل انٹیلی جنس اسسٹنٹ',
  }
};

// Add missing keys to other languages to ensure all translations are available
const ensureTranslations = (translations) => {
  const allKeys = Object.keys(defaultTranslations.en);
  const result = { ...translations };

  Object.keys(translations).forEach(lang => {
    if (lang !== 'en') {
      const langTranslations = { ...translations[lang] };
      allKeys.forEach(key => {
        if (!langTranslations[key]) {
          langTranslations[key] = defaultTranslations.en[key];
        }
      });
      result[lang] = langTranslations;
    }
  });

  return result;
};

const translations = ensureTranslations(defaultTranslations);

const TranslationContext = createContext();

export const TranslationProvider = ({ children }) => {
  const [locale, setLocale] = useState('en');
  const location = useLocation();

  // Extract locale from URL path
  useEffect(() => {
    const pathParts = location.pathname.split('/');
    const pathLocale = pathParts[1];

    if (translations[pathLocale]) {
      setLocale(pathLocale);
    } else {
      setLocale('en'); // default
    }
  }, [location.pathname]);

  const t = (key, fallback = key) => {
    return translations[locale]?.[key] || translations['en'][key] || fallback;
  };

  const value = {
    locale,
    t,
    availableLocales: Object.keys(translations),
    setLocale
  };

  return (
    <TranslationContext.Provider value={value}>
      {children}
    </TranslationContext.Provider>
  );
};

export const useTranslation = () => {
  const context = useContext(TranslationContext);
  if (!context) {
    throw new Error('useTranslation must be used within a TranslationProvider');
  }
  return context;
};